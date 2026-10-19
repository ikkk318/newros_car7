/* ══════════════════════════════════════════════════════════════════
   ui.js —— 任务缓冲、地图库、自检、栈控制、模式切换、登录与启动
   ══════════════════════════════════════════════════════════════════

   ★ 界面不用原生 alert/prompt/confirm（会阻塞事件循环、样式也不一致），
     统一走 core.js 的 uiConfirm / uiPrompt / uiPick / uiMulti / uiPose。

   ★ 最后一行的 DOMContentLoaded 回调是**整页唯一入口**：
     如果它没跑，先看前面四个脚本有没有 SyntaxError。
*/

// ══════════════════════════════════════════════════════════════
//  任务系统（等价终端 [9]）
// ══════════════════════════════════════════════════════════════
async function refreshTasks() {
  try {
    const r = await api('/api/tasks', {});
    if (!r || r.ok === false) {
      // ★ 以前这里直接把失败抛进 catch，每 5 秒刷一条"读取任务失败"，
      //   真正的错因（405/400）反而被淹没。现在只提示一次，并且不打全文。
      _warnOnce('tasks', '读取任务列表失败，请检查网关是否正常');
      return;
    }
    S.tasks = r.tasks || [];
    S.taskRunning = !!r.running;
    // 后端说没在跑，就把"上一次的执行中"清掉 —— 这是界面永远卡在
    // "执行中…"的最后一道保险（即便 /api/tasks 之前一直失败，
    // 只要恢复成功后端给出 running=false，按钮就能立刻变回"开始执行"）
    if (!S.taskRunning && S._wasRunning) {
      pushLog('info', '任务已结束');
    }
    S._wasRunning = S.taskRunning;
    renderTasks();
  } catch (e) {
    _warnOnce('tasks', `读取任务列表失败（${e.message}），稍后自动重试`);
  }
}

// 同一类警告只打一次，避免每几秒刷屏把日志淹掉
// 参数：key - 去重键；text - 提示内容
// 返回：无
function _warnOnce(key, text) {
  if (!S._warned) S._warned = new Set();
  if (S._warned.has(key)) return;
  S._warned.add(key);
  pushLog('warn', text);
  // 5 分钟后允许再提示一次（长时间修不好时还能看见）
  setTimeout(() => S._warned.delete(key), 300000);
}

function renderTasks() {
  const box = $('taskList');
  const list = S.tasks || [];
  const btn = $('btnTaskRun');
  if (S.taskRunning) {
    btn.textContent = '执行中…';
    btn.classList.add('running');
  } else {
    btn.textContent = '开始执行';
    btn.classList.remove('running');
  }
  if (!list.length) {
    box.innerHTML = '<p class="hint small">任务缓冲为空</p>';
    return;
  }
  box.innerHTML = '';
  list.forEach((t, i) => {
    const d = document.createElement('div');
    d.className = 'wp';
    // ★ 路线要显示出来（以前完全没显示，用户根本不知道这单走不走路线，
    //   也没法改）。点整行 = 改这一单的路线，不用删了重建。
    const rt = t.route
      ? `<span class="kd cargo">经 ${escapeHtml(t.route)}</span>`
      : `<span class="kd">直连</span>`;
    d.innerHTML = `<span class="nm">${escapeHtml(t.pick)} ⇒ ${escapeHtml(t.drop)}</span>`
      + rt
      + `<span class="kd">×${t.loops || 1}</span>`;
    d.title = '点这一行可以改这单走的路线';
    d.addEventListener('click', () => editTaskRoute(i));
    const del = document.createElement('button');
    del.className = 'wp-del';
    del.textContent = '×';
    del.addEventListener('click', (ev) => { ev.stopPropagation(); delTask(i); });
    d.appendChild(del);
    box.appendChild(d);
  });
}

/** 改某一单的路线：只改 route 字段，其它不动（走 delete + add 两条接口
 *  太脏，会打乱顺序；后端 save 是整表覆盖，所以整体回写一次即可）。 */
async function editTaskRoute(i) {
  const t = (S.tasks || [])[i];
  if (!t) return;
  if (S.taskRunning) {
    pushLog('warn', '任务正在执行，先中止才能改路线');
    return;
  }
  const routes = S.routes || [];
  const counts = S.routeCounts || [];
  const items = [{value: '', label: '不走路线（两点直连）'}];
  routes.forEach((nm, k) => {
    items.push({value: nm, label: nm, sub: `${counts[k] || 0} 个途经点`});
  });
  const v = await uiPick(
    '修改任务路线',
    `${t.pick} ⇒ ${t.drop}\n当前：${t.route ? '经「' + t.route + '」' : '两点直连'}`,
    items, t.route || '', '保存');
  if (v === null) return;
  if (v === (t.route || '')) { pushLog('info', '路线没变'); return; }
  try {
    // 后端有 replace（整表覆盖）：一次请求改写这一条的路线，顺序不变
    const all = (S.tasks || []).slice();
    all[i] = Object.assign({}, t, {route: v});
    const r = await api('/api/tasks', {action: 'replace', tasks: all});
    if (r && Array.isArray(r.tasks)) S.tasks = r.tasks;
    renderTasks();
    pushLog('ok', v ? `已改为经路线「${v}」` : '已改为两点直连');
  } catch (e) { pushLog('warn', `修改路线失败：${e.message}`); }
}
async function addTask() {
  const names = (S.waypoints || []).map((w) => w.name);
  if (names.length < 2) { pushLog('warn', '至少需要 2 个点位才能建搬运任务'); return; }
  const opts = names.map((nm) => ({value: nm, label: nm}));
  // ★ 原本是三次 prompt（取货点 / 放货点 / 循环次数），全改成点选。
  //   放货点的列表里把已选的取货点置灰提示，避免"取=放"这种无意义任务。
  const pick = await uiPick('新增搬运任务 · 第 1 步', '选择取货点', opts, null, '下一步');
  if (!pick) return;
  const dropOpts = opts.map((o) => ({
    value: o.value, label: o.label,
    sub: o.value === pick ? '（已作为取货点）' : '',
  }));
  const defDrop = names.find((n) => n !== pick) || null;
  const drop = await uiPick('新增搬运任务 · 第 2 步', '选择放货点',
                             dropOpts, defDrop, '下一步');
  if (!drop) return;
  if (drop === pick) {
    pushLog('warn', '取货点和放货点不能相同');
    return;
  }
  // ★ 第 3 步：选路线（本轮新增）。
  //   以前网页只能建"两点直连"的搬运任务，终端控制台早就支持 route，
  //   而 /api/tasks 的 add 也一直有 route 字段 —— 中间只是**没做入口**，
  //   等于功能存着但用不了。现在补上，并且把"不用路线"放在列表第一位，
  //   大多数场景直接点确定。
  const route = await pickRouteForTask(pick, drop);
  if (route === null) return;            // 用户取消（空串 = 明确不用路线）
  const loops = await uiNumber('新增搬运任务 · 第 4 步',
                               `${pick} ⇒ ${drop}`
                               + (route ? `\n经路线「${route}」` : '\n不走路线，两点直连')
                               + '\n要重复执行几次？', 1, 1, 99);
  if (!loops) return;
  try {
    const r = await api('/api/tasks', {action: 'add', pick, drop, loops, route});
    S.tasks = r.tasks || [];
    renderTasks();
    pushLog('ok', `已加入任务：${pick} ⇒ ${drop}`
                  + (route ? `（经 ${route}）` : '') + ` ×${loops}`);
  } catch (e) { pushLog('warn', `添加任务失败：${e.message}`); }
}

/** 任务选路线：返回路线名（''=不走路线）或 null（用户取消）。
 *  ★ 只列"起点/终点能对得上"的路线会更贴心，但车间里路线常常是
 *    从某个路口绕过去，起点终点未必等于取放点，所以这里全列出来，
 *    只把首尾不匹配的路线在副标题里标一下，不硬过滤。 */
async function pickRouteForTask(pick, drop) {
  const routes = S.routes || [];
  const counts = S.routeCounts || [];
  const items = [{value: '', label: '不走路线（两点直连）', sub: '最快，但可能撞到货架'}];
  routes.forEach((nm, i) => {
    const n = counts[i] || 0;
    items.push({value: nm, label: nm, sub: `${n} 个途经点`});
  });
  const v = await uiPick(
    '新增搬运任务 · 第 3 步',
    routes.length
      ? `${pick} ⇒ ${drop}\n要经过某条路线吗？（路线会在取货点之后、放货点之前依次走过）`
      : `${pick} ⇒ ${drop}\n当前没有任何路线，将两点直连导航。`,
    items, '', '下一步');
  return v;                               // null=取消；''=不用路线
}

async function delTask(i) {
  try {
    const r = await api('/api/tasks', {action: 'delete', index: i});
    S.tasks = r.tasks || [];
    renderTasks();
    pushLog('info', r.message || '已删除任务');
  } catch (e) { pushLog('warn', `删除任务失败：${e.message}`); }
}

async function runTasks() {
  if (S.taskRunning) {
    const yes = await uiConfirm('中止任务', '任务正在执行，确定要中止吗？', '中止');
    if (!yes) return;
    try {
      // ★ 必须走 /api/tasks 的 stop 动作，不能只发一条裸命令。
      //   只发 cancel_task 的话，task_manager 那边停了，但网关的任务线程
      //   还在等状态、task_running 一直是 true —— 于是"中止了还是显示执行中"，
      //   而且再也开不了新任务。stop 动作会同时置停止事件 + 发取消命令。
      const r = await api('/api/tasks', {action: 'stop'});
      S.taskRunning = false;
      renderTasks();
      pushLog('warn', r.message || '已中止任务');
      // 立刻刷一次真实状态，别等 5 秒定时器（用户刚点了按钮，要即时反馈）
      refreshTasks();
    } catch (e) { pushLog('warn', `中止失败：${e.message}`); }
    return;
  }
  try {
    const r = await api('/api/tasks', {action: 'start'});
    S.taskRunning = true;
    renderTasks();
    pushLog('ok', r.message || '已开始执行任务');
  } catch (e) { pushLog('warn', `启动任务失败：${e.message}`); }
}

// ══════════════════════════════════════════════════════════════
//  地图库
// ══════════════════════════════════════════════════════════════
async function refreshMaps() {
  try {
    const r = await api('/api/maps', {});
    S.maps = r.maps || [];
    S.currentMap = r.current || '';
    // info：{地图名: {points, saved_at, has_routes}} —— 只给选地图弹框做副标题，
    // 老后端不返回这个字段时保持 {}，界面照常能用（不会因为少字段崩掉）
    S.mapInfo = r.info || {};
    renderMaps();
  } catch (e) { pushLog('warn', `读取地图库失败：${e.message}`); }
}

function renderMaps() {
  const box = $('mapList');
  const list = S.maps || [];
  if (!list.length) {
    box.innerHTML = '<p class="hint small">地图库为空</p>';
    return;
  }
  box.innerHTML = '';
  for (const nm of list) {
    const d = document.createElement('div');
    d.className = 'wp';
    const cur = nm === S.currentMap;
    d.innerHTML = `<span class="nm">${escapeHtml(nm)}${cur ? '（当前）' : ''}</span>`;
    const sw = document.createElement('button');
    sw.className = 'btn-mini';
    sw.textContent = '切换';
    sw.addEventListener('click', (ev) => { ev.stopPropagation(); switchMap(nm); });
    d.appendChild(sw);
    if (!cur) {
      const del = document.createElement('button');
      del.className = 'wp-del';
      del.textContent = '×';
      del.addEventListener('click', (ev) => { ev.stopPropagation(); deleteMap(nm); });
      d.appendChild(del);
    }
    box.appendChild(d);
  }
}

async function switchMap(name) {
  try {
    const r = await api('/api/map_switch', {name});
    S.currentMap = name;
    renderMaps();
    pushLog('warn', r.message || `已选择地图 ${name}`);
    // ★ 只刷任务缓冲，**不刷**点位和路线。
    //   原因：换图在 ROS 2 里必须重启 map_server，而 waypoint_node 的文件
    //   路径是启动时注入的 —— 只改 current_map 的话，点位/路线服务读的
    //   还是上一张图的文件。这时去刷反而会把"旧图的路线"显示成新图的，
    //   用户点进去就报"路线不存在"，看着像数据丢了。
    //   任务缓冲不一样：它由网关按 current_map 直接读文件，切了就是新的。
    S.picked = null;
    refreshTasks();
  } catch (e) { pushLog('warn', `切换地图失败：${e.message}`); }
}

async function deleteMap(name) {
  const yes = await uiConfirm('删除地图', `确定删除地图「${name}」？此操作不可恢复。`,
                              '删除');
  if (!yes) return;
  try {
    const r = await api('/api/map_delete', {name});
    pushLog(r.ok ? 'ok' : 'warn', r.message || '已删除');
    refreshMaps();
  } catch (e) { pushLog('warn', `删除地图失败：${e.message}`); }
}

// ══════════════════════════════════════════════════════════════
//  自检 / 恢复
// ══════════════════════════════════════════════════════════════
async function runCheck() {
  const out = $('checkOut');
  out.textContent = '检查中…';
  try {
    const r = await api('/api/check', {});
    const lines = (r.items || []).map((it) =>
      `${it.ok ? '✔' : '✘'} ${it.item}  ${it.message}`);
    out.textContent = lines.join('\n');
    pushLog(r.ok ? 'ok' : 'warn',
           r.ok ? '自检全部通过' : '自检有项目未通过');
  } catch (e) {
    out.textContent = '自检失败：' + e.message;
    pushLog('warn', `自检失败：${e.message}`);
  }
}

async function doResume() {
  try {
    await api('/api/resume', {});
    pushLog('warn', '已请求恢复');
  } catch (e) { pushLog('warn', `恢复失败：${e.message}`); }
}

// ══════════════════════════════════════════════════════════════
//  工作模式 / 栈起停（单命令体验的核心）
// ══════════════════════════════════════════════════════════════
async function refreshStack() {
  try {
    const r = await api('/api/stack');
    applyStack(r.stack || {});
  } catch (e) { /* 静默：还没登录或后端忙 */ }
}

function applyStack(st) {
  S.stack = st;
  const badge = $('stackBadge');
  if (badge) {
    const running = st.running === true;
    badge.textContent = running ? (st.mode_text || st.mode) : '未启动';
    badge.className = 'badge '
      + (running ? (st.mode === 'mapping' ? 'warn' : 'ok') : 'unk');
  }
  const hint = $('stackHint');
  if (hint) {
    if (!st.manage) {
      hint.textContent = '当前网关不管理栈（manage_stack=false），请手工启动 ROS 栈。';
    } else if (st.error) {
      hint.textContent = `上次操作出错：${st.error}`;
    } else if (st.running === true) {
      hint.textContent = `正在运行：${st.mode_text}（PID ${st.pid}）。`
        + `遥控话题 ${st.teleop_topic || '—'}`;
    } else {
      hint.textContent = '点上面按钮即可拉起来整套 ROS 栈，不用回终端敲命令。';
    }
  }
  // 模式跟随后端返回的真实模式（不是前端自己猜的）
  if (st.running === true && st.mode && st.mode !== S.mode) applyMode(st.mode);
}

async function startStack(mode) {
  // 导航模式需要地图。**不需要建图**——地图库里已有的图直接就能用来导航。
  // ★ 这里修过一个很坏的问题：以前 `!S.maps.length` 就提示"还没有地图，
  //   请先建图"。而 S.maps 是 POST /api/maps 取回来的，后端那个路由当时
  //   只注册了 GET，所以永远 405 → maps 永远是空 → 用户被强制先去建图，
  //   建完图再点导航还是同样的提示，彻底进不去导航模式。
  //   后端已改成 GET/POST 都收；这里再加一层兜底：本地列表为空时，
  //   不直接下结论，而是再问一次后端，仍为空才判定"真的没图"。
  let map = '';
  if (mode === 'nav') {
    if (!S.maps || !S.maps.length) { await refreshMaps(); }
    let maps = S.maps || [];
    if (!maps.length) {
      // 可能是上一次请求失败过（网络抖动、刚启动还没就绪），再试一次
      await refreshMaps();
      maps = S.maps || [];
    }
    const nameOf = (m) => (typeof m === 'string' ? m : (m.name || ''));
    if (!maps.length) {
      pushLog('error', '地图库确实是空的（后端返回 0 张图）。'
        + '请先点「开始建图」，建完保存地图后再启动导航。');
      return;
    }
    // ★ 无论有几张图都弹一次选择框。
    //   只有一张时以前是"自动用、不打扰"，但用户明确要求改成"每次都要确认"，
    //   理由很实在：建图和导航是两个按钮挨着，误点「启动导航」时如果连地图
    //   都不问就直接把栈拉起来，人是没有任何撤销机会的；多一个确认框就能拦住。
    //   而且以后地图变多，这个交互长什么样用户已经熟悉了，不用再改习惯。
    const items = maps.map((m) => {
      const nm = nameOf(m);
      const inf = (S.mapInfo || {})[nm] || {};
      const bits = [];
      if (inf.points) bits.push(`${inf.points} 个点位`);
      if (inf.has_routes) bits.push('含路线');
      if (inf.saved_at) bits.push(inf.saved_at);
      return {
        value: nm,
        label: nm + (nm === S.currentMap ? '（当前加载）' : ''),
        sub: bits.join(' · '),
      };
    });
    // 默认选中"当前正在用的那张"；没有就用第一张（后端已按名字排序，稳定）
    const def = (S.currentMap && maps.some((m) => nameOf(m) === S.currentMap))
      ? S.currentMap : nameOf(maps[0]);
    map = await uiPick(
      '选择导航地图',
      maps.length === 1
        ? '地图库里有 1 张图，确认用这张启动导航'
        : `地图库里有 ${maps.length} 张图，点一张用来导航`,
      items, def, '启动导航');
    if (!map) return;                       // 用户取消 → 什么都不做
  }
  const label = mode === 'mapping' ? '建图' : '导航';
  pushLog('warn', `正在启动${label}栈…（Nav2 激活约需 15~40 秒，请稍候）`);
  try {
    const r = await api('/api/stack_start', {mode, map});
    pushLog(r.ok ? 'ok' : 'error', r.message || (r.ok ? '已启动' : '启动失败'));
    if (r.stack) applyStack(Object.assign({manage: true}, r.stack));
    if (r.ok && mode === 'nav') {
      // ★ 栈刚起来时 waypoint_node 才上线，这时立刻补一次点位/路线/任务。
      //   以前只刷了点位，路线要等 15 秒的定时器（或者用户去点「新建」），
      //   用户就以为"保存的路线没加载进来"。
      refreshWaypoints();
      refreshRoutes();
      refreshTasks();
    }
  } catch (e) {
    pushLog('error', `启动失败：${e.message}`);
  }
}

async function stopStack() {
  const yes = await uiConfirm('停止 ROS 栈', '确定要停止整套 ROS 栈吗？车会立即停下。',
                              '停止');
  if (!yes) return;
  pushLog('warn', '正在停止栈…');
  try {
    const r = await api('/api/stack_stop', {});
    pushLog(r.ok ? 'ok' : 'error', r.message || '已停止');
    setTimeout(refreshStack, 800);
  } catch (e) {
    pushLog('error', `停止失败：${e.message}`);
  }
}

// ══════════════════════════════════════════════════════════════
//  建图模式（只在 mode=mapping 时启用）
// ══════════════════════════════════════════════════════════════
function applyMode(mode) {
  const mapping = mode === 'mapping';
  const changed = S.mode !== mode;
  S.mode = mode;
  // 建图时这些都没意义：没有导航栈、没有任务层、点位也没和地图绑好
  const hideInMapping = ['cardNav', 'cardWp', 'cardRoute', 'cardTask', 'cardCargo'];
  for (const id of hideInMapping) {
    const el = $(id);
    if (el) el.style.display = mapping ? 'none' : '';
  }
  const cm = $('cardMapping');
  if (cm) cm.style.display = mapping ? '' : 'none';
  if (mapping && changed) {
    pushLog('warn', '当前是【建图模式】：遥控直接驱动底盘，导航/任务/搬运已隐藏');
  }
  // ★ 关键收尾：换模式 = 地图和坐标系都换了，前端必须把"上一张图上的东西"
  //   全部作废，否则就是用户报的"重影 + 小车停在切换前的位置"。
  //   这里不依赖后端的 map_epoch 事件（那是异步的，慢一帧就够看到错位），
  //   本地状态自己先清一遍；后端 epoch 到了再补一次（幂等）。
  if (changed) {
    S.picked = null;
    const pp = $('pickedPos');
    if (pp) {
      pp.textContent = '未选取位置';
      pp.classList.remove('set');
    }
    // ★ 不在这里清 map/img：模式刚切时旧栈可能还在退、新栈还没发图，
    //   清早了会闪一下黑屏。真正的清理时机有两个：
    //     ① 后端 map_epoch 事件（换栈完成，最准）
    //     ② 新地图元信息到达（applyState 里）
    //   这里只清"跨图没有意义"的业务状态（选中点/预览）。
    S.preview = null;
    S.autoCenterDone = false;   // 新图到达时重新适配视图
  }
}

async function saveMap() {
  // ★ 默认名用日期时间生成，建图当天保存基本可以直接点确定。
  const d = new Date();
  const stamp = `${d.getMonth() + 1}月${d.getDate()}日`
              + `${String(d.getHours()).padStart(2, '0')}时`;
  const name = await uiPrompt('保存地图',
                              '给这份地图起个名字（可用默认值，如 厂房A）',
                              '地图' + stamp, '保存');
  if (!name) return;
  pushLog('info', '正在保存地图（map_saver_cli，最多等 45 秒）…');
  try {
    const r = await api('/api/map_save', {name});
    pushLog(r.ok ? 'ok' : 'error', r.message || (r.ok ? '地图已保存' : '保存失败'));
    if (r.ok) refreshMaps();
  } catch (e) {
    pushLog('error', `保存失败：${e.message}`);
  }
}

// ══════════════════════════════════════════════════════════════
//  退出：停栈 + 关网关
// ══════════════════════════════════════════════════════════════
async function shutdownAll() {
  // ★ 要说清楚"退出"和"停止"的区别，否则用户会以为点错了东西。
  const yes = await uiConfirm(
    '退出并关闭车',
    '会停掉整套 ROS 栈（仿真/导航/业务节点）并把网页服务也一起关掉。\n'
    + '之后要再用，得重新在终端启动网关。只是刷新页面的话不需要点这个。',
    '确认退出');
  if (!yes) return;

  // 先还控制权，让别人能接手
  try {
    if (S.hasLease) {
      await api('/api/lease', {action: 'release', who: S.who});
      S.hasLease = false;
    }
  } catch (_) {}

  pushLog('warn', '正在停止并退出（最多等 12 秒）…');
  try {
    const r = await api('/api/shutdown', {who: S.who, stop_stack: true});
    pushLog(r.ok ? 'ok' : 'error', r.message || '已发送退出请求');
  } catch (e) {
    // 网关退出时连接会被掐断，fetch 报 Failed to fetch 是正常的，
    // 不能把它显示成"退出失败"，否则用户会反复点。
    pushLog('info', '网关已断开（退出成功时出现这个提示是正常的）');
  }

  // 清掉 WS 重连定时器，避免它指着一个已经死掉的网关狂重连
  if (S.wsRetry) { clearTimeout(S.wsRetry); S.wsRetry = null; }
  try { if (S.ws) S.ws.close(); } catch (_) {}
  S.shutdown = true;

  // 给用户一个明确的结束态，别让页面看起来"还在工作中"
  const hint = $('stackHint');
  if (hint) hint.textContent = '已退出。要再用请重新启动网关。';
  const badge = $('stackBadge');
  if (badge) { badge.textContent = '已退出'; badge.className = 'badge unk'; }
}

// ══════════════════════════════════════════════════════════════
//  启动
// ══════════════════════════════════════════════════════════════
async function doLogin() {
  const t = $('tokenInput').value;
  try {
    await api('/api/login', {token: t});
    S.token = t;
    localStorage.setItem('agv_token', t);
    $('login').classList.add('hidden');
    $('app').classList.remove('hidden');
    boot();
  } catch (e) {
    $('loginErr').textContent = e.message || '口令错误';
  }
}

function boot() {
  // 每台设备一个随机标识：控制权锁靠它区分"是我还是别的手机"
  S.who = localStorage.getItem('agv_who');
  if (!S.who) {
    S.who = 'dev-' + Math.random().toString(36).slice(2, 10);
    localStorage.setItem('agv_who', S.who);
  }
  resizeCanvas();
  bindDpad();
  bindMap();
  connectWS();
  refreshStack();                          // 先问后端：栈起来了吗？什么模式？
  refreshWaypoints();
  refreshRoutes();
  refreshTasks();
  refreshMaps();
  setInterval(refreshStack, 4000);         // 栈状态会自己变（Nav2 激活要时间）
  setInterval(refreshWaypoints, 15000);    // 定时刷新点位（别处可能新增）
  // ★★ 路线也必须定时刷新（本轮修的真机 bug）。
  //   以前 refreshRoutes() 只在 boot() 里调一次：打开页面那一刻栈还没起，
  //   /agv/store 服务不存在 → 后端 503 → 前端 catch 里只打一行 warn，
  //   S.routes 一直是空 → 路线卡片永远显示"暂无路线"。
  //   之后除非用户去点「新建」，否则**再也不会重新读一次** ——
  //   这就是用户说的"保存的路线加载进网页不显示，得新建一个路线才能
  //   把之前的路线显示出来"。加个定时器，栈一起来就会自动补上。
  setInterval(refreshRoutes, 15000);
  setInterval(refreshTasks, 5000);         // 任务缓冲较活跃，刷得勤一点

  // ★ 控制权心跳续期：每 20 秒轻轻拍一下，告诉后端"我还在用"。
  //   旧版只有发速度时才续期，用户拿到锁后想几秒（3 秒）就过期，
  //   再按方向键就没锁了，得反复点「取得控制权」——这是最影响手感的问题。
  //   现在锁的超时放宽到 120 秒，且这里持续续期，正常操作完全不会掉。
  //   注意要在页面可见时才续：切到后台的页面不该长期占着车。
  setInterval(() => {
    if (!S.hasLease) return;
    if (document.hidden) return;
    api('/api/lease', {action: 'renew', who: S.who}).then((r) => {
      if (!r.ok) { S.hasLease = false; }      // 被接管了，静默认输，由状态刷新提示
    }).catch(() => {});
  }, 20000);

  // 页面重新可见时立刻续一次（手机切回来往往已经过了很久）
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden && S.hasLease) {
      api('/api/lease', {action: 'renew', who: S.who}).catch(() => {});
    }
  });

  // ★ 真正离开页面（关标签 / 点退出）时，把控制权还回去，别一直占着。
  //   这里只用 sendBeacon 发 release —— 不在这里停栈！
  //   因为刷新页面（F5）也会触发 beforeunload，如果顺手停栈，
  //   用户每次刷新车都会关一次，反而更烦。
  //   停栈是「退出」按钮的事，要用户明确点。
  window.addEventListener('beforeunload', () => {
    if (!S.hasLease) return;
    try {
      const url = `/api/lease?token=${encodeURIComponent(S.token || '')}`;
      const blob = new Blob(
        [JSON.stringify({action: 'release', who: S.who})],
        {type: 'application/json'});
      navigator.sendBeacon(url, blob);
    } catch (_) { /* 走了就算了 */ }
  });

  pushLog('info', `本设备标识 ${S.who}`);
}

function bindUI() {
  $('loginBtn').addEventListener('click', doLogin);
  $('tokenInput').addEventListener('keydown', (e) => {
    if (e.key === 'Enter') doLogin();
  });

  $('btnPanic').addEventListener('click', async () => {
    try {
      await api('/api/stop', {});
      pushLog('error', '已急停');
    } catch (e) { pushLog('warn', `急停失败：${e.message}`); }
  });

  $('btnLease').addEventListener('click', async () => {
    try {
      if (S.hasLease) {
        await api('/api/lease', {action: 'release', who: S.who});
        S.hasLease = false;
        pushLog('info', '已交还控制权');
      } else {
        await api('/api/lease', {action: 'acquire', who: S.who});
        S.hasLease = true;
        pushLog('ok', '已取得控制权');
      }
    } catch (e) {
      pushLog('warn', e.message || '操作失败');
    }
    // 立即刷新一次状态显示
    if (S.snap) applyState(S.snap);
  });

  $('spdRange').addEventListener('input', () => {
    $('spdVal').textContent = fmt(curSpeed());
  });

  $('btnLift').addEventListener('click', async () => {
    // ★ 不再在本地 pushLog 一条：后端 api_cargo 已经记录了
    //   "顶升开始 → 顶升完成/失败" 两条事件日志，前端再记一遍就是双份，
    //   日志面板里同一个动作出现两次，回看时反而更乱。
    //   这里只兜住"请求根本没发出去"的情况（那时后端不会有日志）。
    try {
      await api('/api/cargo', {command: 'lift'});
    } catch (e) { pushLog('warn', `顶升请求失败：${e.message}`); }
  });

  $('btnLower').addEventListener('click', async () => {
    try {
      await api('/api/cargo', {command: 'lower'});
    } catch (e) { pushLog('warn', `下降请求失败：${e.message}`); }
  });

  $('btnReloc').addEventListener('click', relocalize);

  $('btnCancel').addEventListener('click', async () => {
    try {
      await api('/api/command', {command: 'cancel_task'});
      pushLog('warn', '已请求取消任务');
    } catch (e) { pushLog('warn', `取消失败：${e.message}`); }
  });

  $('btnClearLog').addEventListener('click', () => {
    S.logs = []; S._seenLogs = new Set(); renderLog();
  });

  // ── 点位管理 ──
  $('btnPickCargo').addEventListener('click', () => saveWaypoint('cargo'));
  $('btnPickNav').addEventListener('click', () => saveWaypoint('nav'));
  $('btnWpManual').addEventListener('click', addWaypointManual);

  // ── 路线 ──
  $('btnRouteNew').addEventListener('click', newRoute);

  // ── 任务系统 ──
  $('btnTaskRun').addEventListener('click', runTasks);
  $('btnTaskAdd').addEventListener('click', addTask);
  $('btnTaskClear').addEventListener('click', async () => {
    const yes = await uiConfirm('清空任务缓冲', '确定清空全部待执行任务？', '清空');
    if (!yes) return;
    try {
      const r = await api('/api/tasks', {action: 'clear'});
      S.tasks = []; renderTasks();
      pushLog('warn', '任务缓冲已清空');
    } catch (e) { pushLog('warn', `清空失败：${e.message}`); }
  });

  // ── 地图库 ──
  $('btnMaps').addEventListener('click', refreshMaps);

  // ── 系统 ──
  $('btnCheck').addEventListener('click', runCheck);
  $('btnResume').addEventListener('click', doResume);

  // ── 建图 ──
  $('btnMapSave').addEventListener('click', saveMap);

  // ── 工作模式 / 栈 ──
  $('btnStartNav').addEventListener('click', () => startStack('nav'));
  $('btnStartMap').addEventListener('click', () => startStack('mapping'));
  $('btnStackStop').addEventListener('click', stopStack);
  $('btnStackReload').addEventListener('click', refreshStack);
  $('btnShutdown').addEventListener('click', shutdownAll);

  // 地图浮动工具
  document.querySelectorAll('.maptools button').forEach((b) => {
    b.addEventListener('click', () => {
      const act = b.dataset.act;
      if (act === 'zoomIn')  S.view.scale = clamp(S.view.scale * 1.3, 4, 400);
      if (act === 'zoomOut') S.view.scale = clamp(S.view.scale / 1.3, 4, 400);
      if (act === 'reset') {
        S.autoCenterDone = false;
        S.follow = true;
        $('btnFollow').classList.add('on');
        maybeCenter();
      }
      if (act === 'follow') {
        S.follow = !S.follow;
        $('btnFollow').classList.toggle('on', S.follow);
      }
      draw();
    });
  });

  // 手机页签：地图/操作/日志 三选一
  document.querySelectorAll('.tabs button').forEach((b) => {
    b.addEventListener('click', () => {
      document.querySelectorAll('.tabs button').forEach((x) => x.classList.remove('on'));
      b.classList.add('on');
      const tab = b.dataset.tab;
      const side = document.querySelector('.side');
      // 地图页签：侧栏整体隐藏（CSS 靠 [data-tab="ops"/"log"] 控制显示）
      side.dataset.tab = (tab === 'map') ? 'none' : tab;
      if (tab === 'map') {
        // 隐藏侧栏时复一次位，让 canvas 拿到正确尺寸
        setTimeout(resizeCanvas, 50);
      }
    });
  });

  window.addEventListener('resize', resizeCanvas);
  window.addEventListener('orientationchange', () => setTimeout(resizeCanvas, 200));

  // 页面重新可见时补一次地图（手机切回前台）
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden) { loadMap(); draw(); }
  });
}

document.addEventListener('DOMContentLoaded', () => {
  bindUI();
  // 已存口令则尝试直连（省去每次输入）
  const saved = localStorage.getItem('agv_token');
  if (saved) {
    $('tokenInput').value = saved;
    api('/api/login', {token: saved}).then(() => {
      S.token = saved;
      $('login').classList.add('hidden');
      $('app').classList.remove('hidden');
      boot();
    }).catch(() => { /* 口令失效：留在登录页 */ });
  }
});

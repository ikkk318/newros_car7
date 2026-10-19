/* ══════════════════════════════════════════════════════════════════
   console.js —— WebSocket 状态推送、遥控方向盘、点位与路线
   ══════════════════════════════════════════════════════════════════

   ★ 遥控的三条纪律（破一条就会"车不停"或"抢不动"）
     ① 松手必须发 0（stopTeleop），不能只依赖后端看门狗；
     ② 页面切到后台/失焦要立刻停（visibilitychange / blur）；
     ③ 控制权不在自己手上时不要抢发，409 会让状态更乱。
*/

// ══════════════════════════════════════════════════════════════
//  WebSocket
// ══════════════════════════════════════════════════════════════
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  S.ws = ws;

  ws.onopen = () => {
    // 重连成功：把"断开计时器"停掉。
    // ★ 只有**提示过断开**（超过 5 秒）才补一条"已恢复" ——
    //   短抖两头都不写日志，否则日志里成对刷"断开/连上"。
    if (S._wsDownTimer) { clearTimeout(S._wsDownTimer); S._wsDownTimer = null; }
    if (S._wsWasDown && S._wsDownLogged) pushLog('ok', '与小车重新连上');
    S._wsWasDown = false;
    S._wsDownLogged = false;
    S.connected = true;
    S._wsFails = 0;
    setChip('chipLink', '已连接', 'ok');
  };
  ws.onclose = () => {
    S.connected = false;
    setChip('chipLink', '已断开', 'err');
    // ★ 已经点过「退出」就别再重连了：网关都没了，重连只会每 2 秒报一次
    //   连接失败，用户看着像"退不干净"。
    if (S.shutdown) { pushLog('info', '已退出，停止重连'); return; }
    // ★ 断线重连以前每次都 pushLog，网线一松就在日志里刷几十条"连接断开"，
    //   真正的业务日志全被冲走。现在的策略：
    //     · 只把状态放到顶部芯片（红了就代表断了），日志不刷；
    //     · 第一次断线提示一次，之后静默重试；
    //     · 重试间隔从 2 秒逐步拉长到最多 10 秒，避免疯狂打网关。
    S._wsFails = (S._wsFails || 0) + 1;
    S._wsWasDown = true;
    // ★ 短抖不写日志（真车实测踩到的坑）：
    //   手机 WiFi 每隔 10~26 秒会掉一次线，2~3 秒内自动重连回来。
    //   原来每次断线都 pushLog（onopen 里 _wsFails 清零 → 下次又算"第一次"），
    //   结果日志里"连接断开 / 重新连上"成对出现十几遍，把顶升、任务的
    //   业务日志全冲走了 —— 用户翻回去只看到一堆断线。
    //   现在的规则：**断开持续超过 5 秒才算真故障**，才提示一次；
    //   5 秒内重连成功就静默（顶部芯片仍然会短暂变红，状态是诚实的）。
    if (!S._wsDownTimer) {
      S._wsDownTimer = setTimeout(() => {
        S._wsDownTimer = null;
        S._wsDownLogged = true;
        pushLog('warn', '与小车连接断开，正在自动重连…');
      }, 5000);
    }
    if (S._wsFails === 10) {
      pushLog('error', '连接持续失败（已重试 10 次），请检查车机是否开机、'
        + 'WiFi 是否在同一局域网');
    }
    const delay = Math.min(2000 * S._wsFails, 10000);
    S.wsRetry = setTimeout(connectWS, delay);
  };
  ws.onerror = () => { /* onclose 会跟进处理 */ };

  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch (_) { return; }
    if (m.type === 'state') {
      S.snap = m.data;
      applyState(m.data);
    } else if (m.type === 'map_epoch') {
      // ★ 换栈事件：必须**先于** map_changed 处理（后端也是这个顺序）。
      //   这里是唯一会作废旧地图/旧位姿的地方，漏了就会出现重影和
      //   "小车停在切换前的位置"。
      onMapEpoch(m.epoch);
    } else if (m.type === 'map_changed') {
      // 同一套栈里地图更新（建图时很频繁）：直接重载，但**不要**清 meta，
      // 否则建图过程中视图会一直被重新适配，屏幕来回跳。
      // ★ 只有真拿到 img 才用；乱序/过期由 loadMap 内部的 seq 保证。
      loadMap();
    } else if (m.type === 'logs') {
      // 服务端日志：合并进本地（去重靠时间去重不精确，简单合并即可）
      mergeServerLogs(m.data || []);
    }
  };
}

function mergeServerLogs(items) {
  let added = false;
  for (const it of items) {
    const key = it.t + '|' + it.text;
    if (!S._seenLogs) S._seenLogs = new Set();
    if (S._seenLogs.has(key)) continue;
    S._seenLogs.add(key);
    S.logs.push(it);
    added = true;
  }
  if (added) {
    S.logs.sort((a, b) => a.t - b.t);
    if (S.logs.length > 400) S.logs.splice(0, S.logs.length - 400);
    renderLog();
  }
}

function setChip(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = 'chip' + (cls ? ' ' + cls : '');
}

function applyState(st) {
  // ★ 先认代次：换栈时后端会把地图/位姿全部作废并 +1，前端必须跟着复位。
  //   放在最前面，是为了让后面的 draw() 用的是复位后的状态，不会先用
  //   旧数据画一帧再清掉（那一帧就是用户看到的"重影"）。
  if (typeof st.map_epoch === 'number') onMapEpoch(st.map_epoch);

  // 模式只在第一次收到状态时应用一次（避免每秒重刷 DOM）
  if (st.mode && S.mode !== st.mode) applyMode(st.mode);

  // ★ 顶部实时数据面板：状态类信息（避障/仲裁/连接/定位…）走这里，
  //   不占运行日志。放在靠前的位置是因为它是用户最常瞥一眼的地方。
  renderBoard(st.board);

  // 建图模式下显示地图尺寸增长情况
  if (S.mode === 'mapping' && st.map_meta) {
    const m = st.map_meta;
    const grow = $('mapGrow');
    if (grow) {
      grow.textContent = `${m.width}×${m.height} 格 · `
        + `${(m.width * m.resolution).toFixed(1)}×`
        + `${(m.height * m.resolution).toFixed(1)} m`;
    }
  }

  // 顶部状态
  const ts = st.task;
  if (ts && ts.state) {
    // ★ 以前直接把 state 串打到芯片上（IDLE / GO_LOAD / TRANSPORT…），
    //   用户看到一堆英文完全不知道车在干什么。这里统一转中文。
    const zh = TASK_STATE_ZH[ts.state] || ts.state;
    // IDLE 时芯片标"空闲"，不说"空闲，等待任务"那么啰嗦
    setChip('chipState', ts.state === 'IDLE' ? '空闲' : zh,
            ts.state === 'IDLE' ? '' : stateCls(ts.state));
    if (ts.state === 'IDLE' || ts.state === 'DONE') {
      // 完成/空闲：统一显示"无进行中任务"，别把上一单的目标名留在上面
      $('taskInfo').textContent = '无进行中任务';
    } else {
      $('taskInfo').textContent = ts.message
        ? `${ts.target || ''} ${ts.message}（${Math.round((ts.progress || 0) * 100)}%）`
        : zh;
    }
  } else {
    setChip('chipState', '空闲', '');
    $('taskInfo').textContent = '无进行中任务';
  }
  setChip('chipMode', st.manual ? '手动' : '自动', st.manual ? 'warn' : '');
  $('btnPanic').classList.toggle('on', !!st.estop);

  // 控制权
  const lease = st.lease || {};
  const mine = lease.owner && lease.owner === S.who;
  const wasMine = S.hasLease;
  S.hasLease = !!mine;
  // ★ 从"我有"变成"我没有"时，必须明确告诉用户，不能静默。
  //   旧版这里只是把按钮文字改掉，用户正在按方向键时毫无反馈，
  //   只觉得"车怎么不走了"，然后疯狂点获取控制权。
  if (wasMine && !mine && !S._leaseWarned) {
    S._leaseWarned = true;
    pushLog('warn', lease.owner
      ? `控制权已被「${lease.owner}」接管，遥控已失效`
      : '控制权已释放，遥控已失效');
  }
  if (mine) S._leaseWarned = false;
  const lb = $('btnLease');
  lb.textContent = mine ? '已控制' : (lease.owner ? '被占用' : '取得控制权');
  lb.classList.toggle('on', !!mine);
  $('leaseHint').textContent = mine
    ? `你正在控制（闲置 ${lease.idle_timeout}s 无操作才释放，按住方向键会持续续期）`
    : (lease.owner ? `控制权在另一台设备（${lease.owner}）手上`
                   : '未取得控制权时遥控无效（防多人抢车）');

  // 跟随小车
  if (S.follow && st.pose) {
    S.view.cx = st.pose.x; S.view.cy = st.pose.y;
  }

  // 坐标读数 + 定位状态
  //
  // ★ 这里以前是 `if (st.pose) { 更新读数 }` —— 拿不到位姿就什么都不做，
  //   于是读数永远停在最后一帧有效值上。切模式后新栈还没定位，用户看到的
  //   x/y/θ 是上一套栈的残留，和小车图标一起"冻"在原地，非常有欺骗性。
  //   现在拿不到位姿就明确显示"定位未就绪"，让用户知道该去重新定位。
  if (st.pose) {
    $('coord').textContent =
      `x ${fmt(st.pose.x)}  y ${fmt(st.pose.y)}  θ ${fmt(st.pose.yaw)}`;
    $('coord').classList.remove('stale');
    S._poseMissing = false;
  } else {
    $('coord').textContent = '定位未就绪（等 AMCL 或点「重新定位」）';
    $('coord').classList.add('stale');
    // 只在第一次丢失时提示一次，避免每 0.25 秒刷屏
    if (!S._poseMissing) {
      S._poseMissing = true;
      pushLog('warn', '拿不到 map→base 位姿：小车位置暂不显示（旧位姿已作废，'
        + '不会停在切换前的位置上误导你）');
    }
  }

  // 地图元信息首次到达：居中
  // ★ 以前只判 `!S.map.meta`：换栈后如果新地图的几何恰好和旧图一样，
  //   这里就不会重新居中；如果不一样，居中条件又要等下一帧 —— 中间那帧
  //   是用"新几何 + 旧视图中心"画的，看着就是图飘走了。改用 autoCenterDone
  //   作为唯一判据（它会在 dropMapCache/换栈时被复位），语义更直白。
  if (st.map_meta) {
    const before = S.map.meta;
    S.map.meta = st.map_meta;
    if (!S.autoCenterDone || !before) maybeCenter();
  }

  draw();
}

// 任务状态码 → 中文（与 TaskState.msg 头部的取值表一一对应）
// ★ 为什么要有这张表：这些状态码是给程序看的（IDLE/GO_LOAD/TRANSPORT…），
//   直接显示在界面上就是一堆英文，用户根本不知道车在干什么。
//   后端 message 字段本来就是中文，但芯片位置空间小，只放短标签。
const TASK_STATE_ZH = {
  IDLE: '空闲',
  GO_LOAD: '取货中',
  LIFTING: '顶升中',
  TRANSPORT: '运输中',
  LOWERING: '放货中',
  DONE: '已完成',
  ERROR: '异常',
  PAUSED: '已暂停',
};

function stateCls(s) {
  if (!s) return '';
  // ★ DONE 是"成功"，不能落进下面的 err 分支（以前 /取消/ 这条正则
  //   会把某些失败态的英文串误判，这里显式先判成功态）
  if (s === 'DONE') return 'ok';
  if (/ERROR|FAILED|CANCEL/.test(s)) return 'err';
  if (/失败|错误|取消|异常/.test(s)) return 'err';
  if (/GO_LOAD|LIFTING|TRANSPORT|LOWERING|PAUSED/.test(s)) return 'warn';
  if (/取货|放货|前往|执行|顶升|下降|运行|运输/.test(s)) return 'warn';
  return 'ok';
}

// ══════════════════════════════════════════════════════════════
//  遥控
// ══════════════════════════════════════════════════════════════
const DIRS = {
  f: [ 1,  0], b: [-1,  0],
  l: [ 0,  1], r: [ 0, -1],
  s: [ 0,  0],
};

let teleTimer = null;
let pressed = new Set();

function curSpeed() { return Number($('spdRange').value) / 100; }

function setTeleop(v, w) {
  S.teleop = {v, w};
  // 以 12 Hz 持续发送：网络抖一下也不会让 ROS 侧判超时停车
  sendTeleop();
}

async function sendTeleop() {
  if (!S.hasLease) return;
  try {
    await api('/api/teleop', {who: S.who, v: S.teleop.v, w: S.teleop.w});
  } catch (e) {
    if (e.code === 'locked_by_other') {
      S.hasLease = false;
      pushLog('warn', '控制权已被其他设备接管');
    } else {
      pushLog('warn', `遥控发送失败：${e.message}`);
    }
  }
}

function startTeleop() {
  if (teleTimer) return;
  teleTimer = setInterval(sendTeleop, 80);   // 12.5 Hz
}

function stopTeleop() {
  if (teleTimer) { clearInterval(teleTimer); teleTimer = null; }
  S.teleop = {v: 0, w: 0};
  if (S.hasLease) sendTeleop();
}

function bindDpad() {
  document.querySelectorAll('.dbtn').forEach((btn) => {
    const dir = btn.dataset.tel;
    const down = (ev) => {
      ev.preventDefault();
      if (!S.hasLease) { pushLog('warn', '请先取得控制权'); return; }
      btn.classList.add('pressed');
      pressed.add(dir);
      const spd = curSpeed();
      if (dir === 's') { setTeleop(0, 0); return; }
      const [lin, ang] = DIRS[dir];
      setTeleop(lin * spd, ang * spd * 2.2);
      startTeleop();
    };
    const up = (ev) => {
      ev.preventDefault();
      btn.classList.remove('pressed');
      pressed.delete(dir);
      if (pressed.size === 0) { stopTeleop(); }
    };
    btn.addEventListener('pointerdown', down);
    btn.addEventListener('pointerup', up);
    btn.addEventListener('pointercancel', up);
    btn.addEventListener('pointerleave', up);
    // 防手机长按弹出右键菜单/选中
    btn.addEventListener('contextmenu', (e) => e.preventDefault());
  });
}

// ══════════════════════════════════════════════════════════════
//  重新定位（等价 RViz 的 2D Pose Estimate）
//
//  ★ 旧实现为什么"用不了"：
//      const pose = S.snap.pose;
//      api('/api/relocalize', {x: pose.x, y: pose.y, yaw: pose.yaw});
//    它把**当前位姿**当成新位姿发回给 AMCL，等于"我在原地告诉 AMCL
//    我在原地"—— 除了重置协方差，什么都没修正。而且没位姿时直接
//    return，所以"定位飘了"（最常见场景）时按钮根本点不动。
//
//  ★ 新流程（对齐 RViz 的语义，但更适合手机）：
//    ① 默认值取"地图上刚选的点" > "小车当前位姿" > 原点
//    ② 弹框里可以改 x/y、拖方向盘改朝向
//    ③ 发出去后如实反馈：没有订阅者（建图模式/AMCL 没起）就直接报错，
//       不能显示"已重新定位"骗人
// ══════════════════════════════════════════════════════════════
async function relocalize() {
  if (S.mode === 'mapping') {
    pushLog('warn', '建图模式没有 AMCL，不需要也不能重新定位；'
      + '建图时位姿从原点开始，建完切到导航模式再定位');
    return;
  }
  const pose = S.snap && S.snap.pose;
  // ★ 没位姿也要能点：这正是最需要重新定位的时候
  const init = S.picked
    ? {x: S.picked.x, y: S.picked.y, yaw: S.picked.yaw}
    : (pose ? {x: pose.x, y: pose.y, yaw: pose.yaw} : {x: 0, y: 0, yaw: 0});

  const v = await uiPose(init, {
    title: '重新定位',
    desc: '告诉 AMCL「车现在实际在哪、朝哪」。\n'
      + (S.picked ? '已用地图上选中的位置作为默认值。' :
         (pose ? '已用当前位姿作为默认值。' : '当前没有位姿，默认原点，请手动填写。'))
      + '\n提示：先在地图上点一下车的真实位置，再点这个按钮最省事。',
    okText: '下发定位',
  });
  if (!v) return;
  try {
    const r = await api('/api/relocalize', {x: v.x, y: v.y, yaw: v.yaw});
    if (r && r.ok === false) {
      pushLog('error', r.message || '重定位没有生效');
      return;
    }
    pushLog('ok', r.message
      || `已下发重定位：x ${fmt(v.x)} y ${fmt(v.y)} θ ${fmt(DEG(v.yaw), 0)}°`);
    pushLog('info', 'AMCL 需要 1~3 秒重新收敛，请观察小车图标是否跳到正确位置；'
      + '如果还是偏，再点一次换个更准的初始点');
  } catch (e) {
    pushLog('warn', `重定位失败：${e.message}`);
  }
}

// ══════════════════════════════════════════════════════════════
//  点位列表（点一下就走，不用打字）
// ══════════════════════════════════════════════════════════════
async function refreshWaypoints() {
  try {
    const r = await api('/api/store', {resource: 'waypoint', action: 'list'});
    const names = r.names || [];
    // ★ list 分支其实**已经**把每个点位带回来了（store_dispatch 的
    //   waypoint/list 会填 items），旧代码却拿着名字再逐个 get 一遍 ——
    //   10 个点位就是 11 次 ROS 服务调用，而这段逻辑每 15 秒跑一次。
    //   真机上表现为"定时刷新那一瞬间遥控变卡"。有 items 就直接用。
    let all = Array.isArray(r.items) ? r.items : [];
    if (!all.length && names.length) {
      // 兼容老后端（list 只给名字）：才退回去逐个取
      for (const n of names) {
        try {
          const g = await api('/api/store',
            {resource: 'waypoint', action: 'get', name: n});
          for (const it of (g.items || [])) all.push(it);
        } catch (_) {}
      }
    }
    S.waypoints = all;
    renderWaypoints();
  } catch (e) {
    _warnOnce('waypoints', `读取点位失败（${e.message}），稍后自动重试`);
  }
}

function renderWaypoints() {
  const box = $('wpList');
  const list = S.waypoints || [];
  if (!list.length) {
    box.innerHTML = '<p class="hint small">还没有点位。'
      + '可以点地图选位置保存，或用上面的「手输」按钮直接填坐标</p>';
    return;
  }
  box.innerHTML = '';
  for (const wp of list) {
    const d = document.createElement('div');
    d.className = 'wp';
    // ★ 把 yaw 显示出来：否则用户依然不知道点位朝向，改错了也看不见
    d.innerHTML = `<span class="nm">${escapeHtml(wp.name)}</span>`
      + `<span class="kd">θ${fmt(DEG(wp.yaw), 0)}°</span>`
      + `<span class="kd ${wp.kind === 'cargo' ? 'cargo' : ''}">`
      + `${wp.kind === 'cargo' ? '搬货' : '导航'}</span>`;
    d.title = '点这一行 = 前往该点';
    // 点整行 = 前往该点（触屏友好，不需要菜单）
    d.addEventListener('click', () => gotoWaypoint(wp.name));
    // 编辑：小笔按钮（避免和"前往"抢点击）
    const ed = document.createElement('button');
    ed.className = 'wp-del';
    ed.textContent = '✎';
    ed.title = '编辑坐标/朝向';
    ed.addEventListener('click', (ev) => {
      ev.stopPropagation();
      editWaypoint(wp.name);
    });
    d.appendChild(ed);
    // 删除需单独按钮，避免误触把点位删掉
    const del = document.createElement('button');
    del.className = 'wp-del';
    del.textContent = '×';
    del.title = '删除点位';
    del.addEventListener('click', (ev) => {
      ev.stopPropagation();
      deleteWaypoint(wp.name);
    });
    d.appendChild(del);
    box.appendChild(d);
  }
}

async function gotoWaypoint(name) {
  try {
    await api('/api/command', {command: 'goto', args: [name]});
    pushLog('ok', `已下发导航：前往 ${name}`);
  } catch (e) {
    pushLog('warn', `导航下发失败：${e.message}`);
  }
}


// ══════════════════════════════════════════════════════════════
//  点位保存（坐标 + 朝向 + 类型，全部点选，不必打字）
// ══════════════════════════════════════════════════════════════
async function saveWaypoint(kind) {
  if (!S.picked) { pushLog('warn', '请先在地图上点选一个位置'); return; }
  const isCargo = kind === 'cargo';

  // ★ 名字给智能默认：搬货点按"货架N"、导航点按"点位N"递增，
  //   大多数情况直接点确定就行，不必碰键盘。
  const prefix = isCargo ? '货架' : '点位';
  const used = new Set((S.waypoints || []).map((w) => w.name));
  let n = 1;
  while (used.has(prefix + n)) n++;

  // ★ 类型也能在这个弹框里改：用户点的是"存为导航点"，但看到弹窗才想起来
  //   这是货架 —— 以前得先取消、再点另一个按钮、再点一次地图。现在直接切。
  const extra = document.createElement('div');
  extra.className = 'field';
  const lb = document.createElement('span');
  lb.textContent = '点位名';
  const nameInp = document.createElement('input');
  nameInp.type = 'text';
  nameInp.value = prefix + n;
  nameInp.setAttribute('aria-label', '点位名');
  extra.append(lb, nameInp);

  let curKind = kind;
  const segWrap = document.createElement('div');
  segWrap.className = 'field';
  const segLb = document.createElement('span');
  segLb.textContent = '类型';
  const seg = document.createElement('div');
  seg.className = 'seg';
  const mkSeg = (val, text) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = text;
    b.addEventListener('click', () => {
      curKind = val;
      paintSeg();
    });
    seg.appendChild(b);
    return b;
  };
  const bCargo = mkSeg('cargo', '搬货点');
  const bNav = mkSeg('nav', '导航点');
  const paintSeg = () => {
    bCargo.classList.toggle('on', curKind === 'cargo');
    bNav.classList.toggle('on', curKind === 'nav');
  };
  paintSeg();
  segWrap.append(segLb, seg);
  extra.appendChild(segWrap);

  // ★ 弹框顺序：名字（身份）→ 坐标 → 朝向 → 类型。
  //   名字放最上面还有一个实际好处：手机键盘弹出来时遮住的是下半屏，
  //   而"输名字"是唯一需要键盘的动作，放最上不会被自己挡住。
  const v = await uiPose(S.picked, {
    title: isCargo ? '存为搬货点' : '存为导航点',
    desc: '坐标和朝向都可以改：拖方向盘调朝向，或者直接输角度',
    head: extra,
    tail: segWrap,
    okText: '保存',
  });
  if (!v) return;
  const name = (nameInp.value || '').trim();
  if (!name) { pushLog('warn', '点位名不能为空'); return; }
  try {
    const r = await api('/api/store', {
      resource: 'waypoint', action: 'save', name,
      x: v.x, y: v.y, yaw: v.yaw, kind: curKind,
    });
    pushLog(r.ok ? 'ok' : 'warn',
           r.message || `已保存点位 ${name}（${curKind === 'cargo' ? '搬货' : '导航'}，`
                        + `朝向 ${fmt(DEG(v.yaw), 0)}°）`);
    if (r.ok) {
      // 保存成功后把选中的标记更新成刚存的值（用户可能接着要在旁边再存一个）
      S.picked = {x: v.x, y: v.y, yaw: v.yaw};
      $('pickedPos').textContent =
        `已选 x ${fmt(v.x)}  y ${fmt(v.y)}  θ ${fmt(DEG(v.yaw), 0)}°`;
      refreshWaypoints();
    }
  } catch (e) { pushLog('warn', `保存失败：${e.message}`); }
}

// ══════════════════════════════════════════════════════════════
//  手动添加点位（不点地图，直接输坐标）
//
//  ★ 为什么要这个入口：现场经常出现"我手上有一张纸上记的坐标"或者
//    "我要在 x=2.35,y=1.80 放个点，但地图上那个位置被货架挡住点不准"
//    的情况。以前只能靠点击地图，精度受缩放和手指粗细限制，
//    远端一个像素就是好几厘米。现在可以精确输入。
// ══════════════════════════════════════════════════════════════
async function addWaypointManual() {
  const pose = S.snap && S.snap.pose;
  const used = new Set((S.waypoints || []).map((w) => w.name));
  let n = 1;
  while (used.has('点位' + n)) n++;

  const extra = document.createElement('div');
  extra.className = 'field';
  const lb = document.createElement('span');
  lb.textContent = '点位名';
  const nameInp = document.createElement('input');
  nameInp.type = 'text';
  nameInp.value = '点位' + n;
  nameInp.setAttribute('aria-label', '点位名');
  extra.append(lb, nameInp);

  let curKind = 'nav';
  const segWrap = document.createElement('div');
  segWrap.className = 'field';
  const segLb = document.createElement('span');
  segLb.textContent = '类型';
  const seg = document.createElement('div');
  seg.className = 'seg';
  const mk = (val, text) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = text;
    b.addEventListener('click', () => { curKind = val; paint(); });
    seg.appendChild(b);
    return b;
  };
  const bc = mk('cargo', '搬货点');
  const bn = mk('nav', '导航点');
  const paint = () => {
    bc.classList.toggle('on', curKind === 'cargo');
    bn.classList.toggle('on', curKind === 'nav');
  };
  paint();
  segWrap.append(segLb, seg);

  // ★ 默认值优先级：地图上刚选的 > 小车当前位姿 > 原点。
  //   用"小车当前位姿"当默认很实用：站在要加点的位置按一下，坐标基本就对。
  const init = S.picked
    ? {x: S.picked.x, y: S.picked.y, yaw: S.picked.yaw}
    : (pose ? {x: pose.x, y: pose.y, yaw: pose.yaw} : {x: 0, y: 0, yaw: 0});

  const v = await uiPose(init, {
    title: '手动添加点位',
    desc: '直接输入坐标与朝向（不需要在地图上点）。默认值已填成'
      + (S.picked ? '地图上刚选的位置' : (pose ? '小车当前位置' : '原点')),
    head: extra,
    tail: segWrap,
    okText: '保存',
  });
  if (!v) return;
  const name = (nameInp.value || '').trim();
  if (!name) { pushLog('warn', '点位名不能为空'); return; }
  if (used.has(name)) {
    const yes = await uiConfirm('点位已存在', `已有点位「${name}」，要覆盖它吗？`, '覆盖');
    if (!yes) return;
  }
  try {
    const r = await api('/api/store', {
      resource: 'waypoint', action: 'save', name,
      x: v.x, y: v.y, yaw: v.yaw, kind: curKind,
    });
    pushLog(r.ok ? 'ok' : 'warn', r.message
      || `已保存点位 ${name}（x ${fmt(v.x)} y ${fmt(v.y)} θ ${fmt(DEG(v.yaw), 0)}°）`);
    if (r.ok) {
      S.picked = {x: v.x, y: v.y, yaw: v.yaw};
      const el = $('pickedPos');
      if (el) {
        el.textContent = `已选 x ${fmt(v.x)}  y ${fmt(v.y)}  θ ${fmt(DEG(v.yaw), 0)}°`;
        el.classList.add('set');
      }
      refreshWaypoints();
      // 手输的点可能在视图外，把视图带过去，否则用户以为没存上
      S.view.cx = v.x; S.view.cy = v.y;
      S.follow = false;
      const fb = $('btnFollow');
      if (fb) fb.classList.remove('on');
      draw();
    }
  } catch (e) { pushLog('warn', `保存失败：${e.message}`); }
}

/** 编辑已有点位（改坐标/朝向/类型/改名）——顺手补上的：
 *  以前点位存歪了只能删掉重存，而删除会把引用它的路线一起弄坏。 */
async function editWaypoint(name) {
  const wp = (S.waypoints || []).find((w) => w.name === name);
  if (!wp) return;
  const extra = document.createElement('div');
  extra.className = 'field';
  const lb = document.createElement('span');
  lb.textContent = '点位名（改名会新增一个点位，旧的不动）';
  const nameInp = document.createElement('input');
  nameInp.type = 'text';
  nameInp.value = name;
  nameInp.setAttribute('aria-label', '点位名');
  extra.append(lb, nameInp);

  let curKind = wp.kind === 'cargo' ? 'cargo' : 'nav';
  const segWrap = document.createElement('div');
  segWrap.className = 'field';
  const segLb = document.createElement('span');
  segLb.textContent = '类型';
  const seg = document.createElement('div');
  seg.className = 'seg';
  const mk = (val, text) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = text;
    b.addEventListener('click', () => { curKind = val; paint(); });
    seg.appendChild(b);
    return b;
  };
  const bc = mk('cargo', '搬货点');
  const bn = mk('nav', '导航点');
  const paint = () => {
    bc.classList.toggle('on', curKind === 'cargo');
    bn.classList.toggle('on', curKind === 'nav');
  };
  paint();
  segWrap.append(segLb, seg);
  extra.appendChild(segWrap);

  const v = await uiPose({x: wp.x, y: wp.y, yaw: wp.yaw}, {
    title: `编辑点位「${name}」`,
    desc: '改完点保存即可覆盖；改名则相当于新建一个点位',
    head: extra,
    tail: segWrap,
    okText: '保存',
  });
  if (!v) return;
  const newName = (nameInp.value || '').trim();
  if (!newName) { pushLog('warn', '点位名不能为空'); return; }
  try {
    const r = await api('/api/store', {
      resource: 'waypoint', action: 'save', name: newName,
      x: v.x, y: v.y, yaw: v.yaw, kind: curKind,
    });
    pushLog(r.ok ? 'ok' : 'warn', r.message || `已更新点位 ${newName}`);
    if (r.ok) refreshWaypoints();
  } catch (e) { pushLog('warn', `更新失败：${e.message}`); }
}

async function deleteWaypoint(name) {
  const yes = await uiConfirm('删除点位', `确定删除点位「${name}」？删除后不可恢复。`,
                              '删除');
  if (!yes) return;
  try {
    const r = await api('/api/store', {resource: 'waypoint', action: 'delete', name});
    pushLog(r.ok ? 'ok' : 'warn', r.message || `已删除 ${name}`);
    refreshWaypoints();
  } catch (e) { pushLog('warn', `删除失败：${e.message}`); }
}

// ══════════════════════════════════════════════════════════════
//  路线
// ══════════════════════════════════════════════════════════════
async function refreshRoutes() {
  try {
    const r = await api('/api/routes', {action: 'list'});
    // ★ 后端读不到服务时会给 503 / ok=false，这时**不能**把本地列表清空
    //   （否则刚显示出来的路线会突然消失）。只有真拿到 names 才覆盖。
    if (!r || r.ok === false || !Array.isArray(r.names)) {
      _warnOnce('routes', '读取路线失败（点位服务 /agv/store 未就绪），稍后自动重试');
      _retryRoutesSoon();               // use 快速补一次，别让用户干等 15 秒
      return;
    }
    S.routes = r.names || [];
    S.routeCounts = r.counts || [];
    renderRoutes(r);
  } catch (e) {
    _warnOnce('routes', `读取路线失败（${e.message}），稍后自动重试`);
    _retryRoutesSoon();
  }
}

/** 路线读取失败后的快速重试（最多 3 次，间隔 2 秒）。
 *  ★ 为什么需要：定时器是 15 秒一次，而"栈刚起来"这个窗口只有几秒。
 *    用户在栈启动后打开页面，第一次读必然 503，然后要干等 15 秒才看到
 *    已经存在的路线 —— 体感就是"路线不显示"。这里失败就补两次。 */
let _routeRetry = 0, _routeRetryTimer = null;
function _retryRoutesSoon() {
  if (_routeRetry >= 3) return;
  if (_routeRetryTimer) return;
  _routeRetry += 1;
  _routeRetryTimer = setTimeout(() => {
    _routeRetryTimer = null;
    refreshRoutes();
  }, 2000);
}

function renderRoutes(r) {
  const box = $('routeList');
  const names = (r && r.names) || [];
  if (!names.length) {
    box.innerHTML = '<p class="hint small">暂无路线</p>';
    return;
  }
  box.innerHTML = '';
  names.forEach((nm, i) => {
    const d = document.createElement('div');
    d.className = 'wp';
    const cnt = (r.counts && r.counts[i]) || 0;
    d.innerHTML = `<span class="nm">${escapeHtml(nm)}</span>`
                + `<span class="kd">${cnt} 点</span>`;
    d.addEventListener('click', () => showRoute(nm));
    box.appendChild(d);
  });
}

async function showRoute(name) {
  try {
    const r = await api('/api/routes', {action: 'get', name});
    const pts = (r.items || []).map((x) => x.name).join(' → ');
    pushLog('info', `路线「${name}」：${pts || '（空）'}`);
    const yes = await uiConfirm(`路线「${name}」`,
                                pts || '（这条路线没有途经点）', '删除路线');
    if (yes) {
      const d = await api('/api/routes', {action: 'delete', name});
      pushLog(d.ok ? 'ok' : 'warn', d.message || '已删除');
      refreshRoutes();
    }
  } catch (e) { pushLog('warn', `读取路线详情失败：${e.message}`); }
}

async function newRoute() {
  const names = (S.waypoints || []).map((w) => w.name);
  if (!names.length) { pushLog('warn', '还没有点位，无法建路线'); return; }
  // ★ 路线名同样给默认值（路线1/路线2…），直接点确定即可
  let n = 1;
  const used = new Set(S.routes || []);
  while (used.has('路线' + n)) n++;
  const routeName = await uiPrompt('新建路线', '给这条路线起个名字（可用默认值）',
                                    '路线' + n, '下一步');
  if (!routeName) return;
  // ★ 关键改动：原本要用户"输入逗号分隔的途经点序列"，
  //   手机上几乎不可能完成。改成按顺序点选，点错再点一下取消。
  const point_names = await uiMulti(
    '选择途经点',
    `路线「${routeName}」：按顺序点选经过的点位，第一个是起点、最后一个是终点`,
    names.map((nm) => ({value: nm, label: nm})),
    '保存路线');
  if (!point_names) return;
  try {
    const r = await api('/api/routes', {action: 'save', name: routeName, point_names});
    pushLog(r.ok ? 'ok' : 'warn', r.message || `已保存路线 ${routeName}`);
    refreshRoutes();
  } catch (e) { pushLog('warn', `保存路线失败：${e.message}`); }
}

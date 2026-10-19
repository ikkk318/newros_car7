/* ══════════════════════════════════════════════════════════════════
   core.js —— 全局状态、HTTP 助手、模态框、日志渲染
   ══════════════════════════════════════════════════════════════════

   ★ 四个前端脚本是**经典脚本**（不是 ES module、不是 IIFE），
     共享同一个全局词法环境。任何一个顶层名字重复声明（const/let），
     整个页面的脚本直接 SyntaxError —— 白屏，且控制台只有一行提示。
     所以：每个顶层符号只在一个文件里声明，加东西前先 grep 四个文件。

   加载顺序（见 index.html 末尾）：core → map → console → ui
      core     状态容器 S、$、api()、uiConfirm/uiPrompt/uiPick/uiMulti/uiPose、日志
      map      canvas、坐标变换、draw*、地图加载与缓存、拖拽缩放、顶部仪表
      console  WebSocket/applyState、遥控方向盘、点位、路线
      ui       任务、地图库、自检、栈控制、模式切换、登录、启动绑定
*/

/* ══════════════════════════════════════════════════════════════════
   AGV 网页控制台前端
   目标：完全替代终端控制台 + 具备 RViz 的地图/障碍/位姿显示。

   ★ 坐标约定（这里错了整张图就会错位，且看着很像"定位漂了"）
   ROS 世界坐标：x 向右、y 向上，单位米。
   Canvas 像素：x 向右、**y 向下**。
   地图 origin 是**左下角**在世界坐标里的位置。
   所以：px = (x - origin_x)/res，py = height - (y - origin_y)/res
   ══════════════════════════════════════════════════════════════════ */

'use strict';

// ─────────────── 全局状态 ───────────────
const S = {
  token: '',            // 访问口令
  who: '',              // 本设备标识（控制权用）
  ws: null,             // WebSocket
  connected: false,
  hasLease: false,
  follow: true,         // 视图跟随小车
  autoCenterDone: false,
  // ★ map 里除了图片和几何，还要记「代次(epoch)」和「请求序号(seq)」：
  //   epoch —— 后端换了一套栈就 +1。它和 map_version 不是一回事：
  //            version 在建图时每 0.8 秒就变（地图在长），epoch 只在换栈时变。
  //   seq   —— 本地连续发了几次 /api/map.png。浏览器可能让先发的请求后到，
  //            不认序号就会把旧图盖到新图上 → 屏幕上出现"重影"。
  map: {img: null, meta: null, version: -1, epoch: -1, seq: 0},
  view: {cx: 0, cy: 0, scale: 40},   // 视图中心（世界坐标）+ 缩放(px/米)
  snap: null,           // 最近一次状态
  teleop: {v: 0, w: 0}, // 当前遥控输出
  logs: [],
  // ★ 顶部实时数据面板：{key: true}，只用来判断"上一帧有哪些格子"，
  //   据此删掉后端已经不再上报的格子（避障解除、连接恢复…）。
  //   真正的文本在 DOM 里，不在 JS 状态里 —— 避免每次刷新都重建字符串。
  board: {},
  waypoints: [],        // 点位列表 [{name,x,y,yaw,kind}]
  routes: [],           // 路线名列表
  routeCounts: [],      // 与 routes 一一对应的途经点数（选路线时显示）
  picked: null,         // 地图上选中的位置 {x,y,yaw}（存点位/重新定位都用它）
  preview: null,        // 弹框里正在调的位置/朝向预览 {x,y,yaw}，关闭弹框即清
  tasks: [],            // 任务缓冲
  taskRunning: false,
  maps: [],             // 地图库（纯地图名数组，按名排序）
  mapInfo: {},          // 地图详情 {名: {points, saved_at, has_routes}}，仅用于选图副标题
  currentMap: '',
  mode: 'nav',
  wsRetry: null,        // WS 重连定时器句柄（退出时要清掉）
  shutdown: false,      // 已点过退出：别再自动重连，否则会对着死网关猛连
};

// ─────────────── 小工具 ───────────────
const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

function fmt(n, d = 2) { return Number(n).toFixed(d); }

function nowStr(ts) {
  const d = new Date((ts || Date.now() / 1000) * 1000);
  return `${String(d.getHours()).padStart(2, '0')}:`
       + `${String(d.getMinutes()).padStart(2, '0')}:`
       + `${String(d.getSeconds()).padStart(2, '0')}`;
}

async function api(path, body) {
  const opt = {method: 'POST', headers: {'Content-Type': 'application/json'}};
  if (body !== undefined) opt.body = JSON.stringify(body);
  const r = await fetch(path, opt);
  let j = {};
  try { j = await r.json(); } catch (_) {}
  if (!r.ok) throw Object.assign(new Error(j.message || `HTTP ${r.status}`), j);
  return j;
}

// ══════════════════════════════════════════════════════════════
//  通用模态框 —— 替代 prompt() / confirm()
//
//  ★ 为什么必须换掉 prompt()
//  这台车是"人拿着手机在车间里走"用的。原版有 8 处 prompt()：
//  存点位要输名字、新建路线要输"逗号分隔的途经点序列"、新增任务要
//  输三个字段……手机上每弹一次系统输入框都要收放键盘，而且中文输入法
//  会盖住半个屏幕，输逗号分隔的点位顺序基本是不可能完成的任务。
//  这里统一成三种交互，全部只用拇指点：
//    uiPick   —— 从列表里点选一个（点位/地图/路线）
//    uiMulti  —— 多选并带顺序编号（途经点序列）
//    uiPrompt —— 只在一处不得不用键盘时用（起名字），且预填默认值
//  uiConfirm 替代 confirm()，样式统一，也不会被浏览器拦。
// ══════════════════════════════════════════════════════════════

let _uiResolve = null;      // 当前模态框的 resolve
let _uiOnOk = null;         // 当前模态框"确定"时的取值函数
let _uiEsc = null;          // Esc / 取消时的行为（默认 resolve(null)）

/** 内部：显示模态框。
 *  参数 opts: {title, desc, body(HTMLElement), okText, cancelText,
 *              onOk(返回 {ok, value}), hideCancel}
 *  返回 Promise：确定→value；取消→null
 */
function _uiShow(opts) {
  return new Promise((resolve) => {
    _uiResolve = resolve;
    _uiOnOk = opts.onOk || (() => ({ok: true, value: true}));
    $('modalTitle').textContent = opts.title || '';
    $('modalDesc').textContent = opts.desc || '';
    const body = $('modalBody');
    body.innerHTML = '';
    if (opts.body) body.appendChild(opts.body);
    $('modalOk').textContent = opts.okText || '确定';
    $('modalCancel').textContent = opts.cancelText || '取消';
    $('modalCancel').style.display = opts.hideCancel ? 'none' : '';
    $('modal').classList.remove('hidden');
    // 有输入框就自动聚焦并选中预填内容，省一次点击
    const inp = body.querySelector('input');
    if (inp) {
      setTimeout(() => { inp.focus(); inp.select && inp.select(); }, 60);
    }
  });
}

function _uiClose(value) {
  $('modal').classList.add('hidden');
  const r = _uiResolve;
  _uiResolve = null;
  _uiOnOk = null;
  if (r) r(value);
}

/** 确认框：返回 true/false（同步语义的 Promise） */
async function uiConfirm(title, desc, okText = '确定') {
  const v = await _uiShow({title, desc, okText, body: null,
                           onOk: () => ({ok: true, value: true})});
  return v === true;
}

/** 单行输入：返回字符串或 null。
 *  只在"必须起名字"时用，且尽量给默认值，让用户直接点确定。 */
async function uiPrompt(title, desc, defVal = '', okText = '确定') {
  const wrap = document.createElement('div');
  const inp = document.createElement('input');
  inp.type = 'text';
  inp.value = defVal;
  inp.placeholder = defVal || '';
  wrap.appendChild(inp);
  const v = await _uiShow({
    title, desc, body: wrap, okText,
    // 输入框里按回车 = 点确定
    onOk: () => {
      const s = (inp.value || '').trim();
      if (!s) { inp.focus(); return {ok: false}; }
      return {ok: true, value: s};
    },
  });
  return v;
}

/** 单选列表：items=[{value,label,sub}]，返回选中的 value 或 null */
async function uiPick(title, desc, items, defVal = null, okText = '确定') {
  if (!items.length) { pushLog('warn', '列表为空'); return null; }
  let sel = defVal != null ? defVal : items[0].value;
  const wrap = document.createElement('div');
  wrap.className = 'opt-list';
  const render = () => {
    wrap.innerHTML = '';
    for (const it of items) {
      const d = document.createElement('div');
      d.className = 'opt' + (it.value === sel ? ' on' : '');
      d.innerHTML = `<span class="tick">✓</span>`
                  + `<span class="nm">${escapeHtml(it.label ?? it.value)}`
                  + (it.sub ? `<span class="sub"> ${escapeHtml(it.sub)}</span>` : '')
                  + `</span>`;
      // 单击即选中；再点确定。双击不必要，减少误触。
      d.addEventListener('click', () => { sel = it.value; render(); });
      wrap.appendChild(d);
    }
  };
  render();
  const v = await _uiShow({
    title, desc, body: wrap, okText,
    onOk: () => ({ok: true, value: sel}),
  });
  return v;
}

/** 多选（带顺序）：点一次加入，再点取消加入。
 *  返回按点击顺序排好的 value 数组，或 null。
 *  ★ 这就是替代"输入逗号分隔的途经点"的方案：点谁谁进队列，
 *    顺序天然就是点击顺序，不用打字也不用记逗号。 */
async function uiMulti(title, desc, items, okText = '确定') {
  if (!items.length) { pushLog('warn', '列表为空'); return null; }
  const picked = [];                 // 按点击顺序
  const wrap = document.createElement('div');
  wrap.className = 'opt-list';
  const render = () => {
    wrap.innerHTML = '';
    for (const it of items) {
      const idx = picked.indexOf(it.value);
      const d = document.createElement('div');
      d.className = 'opt' + (idx >= 0 ? ' on' : '');
      d.innerHTML = (idx >= 0
          ? `<span class="ord">${idx + 1}</span>`
          : `<span class="tick">+</span>`)
        + `<span class="nm">${escapeHtml(it.label ?? it.value)}`
        + (it.sub ? `<span class="sub"> ${escapeHtml(it.sub)}</span>` : '')
        + `</span>`;
      d.addEventListener('click', () => {
        const i = picked.indexOf(it.value);
        if (i >= 0) picked.splice(i, 1);   // 再点一次 = 取消
        else picked.push(it.value);
        render();
      });
      wrap.appendChild(d);
    }
    // 底部提示当前已选顺序
    const tip = document.createElement('p');
    tip.className = 'hint small';
    tip.style.margin = '10px 0 0';
    tip.textContent = picked.length
      ? `已选 ${picked.length} 个：${picked.join(' → ')}`
      : '按顺序点选，点错再点一下取消';
    wrap.appendChild(tip);
  };
  render();
  const v = await _uiShow({
    title, desc, body: wrap, okText,
    onOk: () => {
      if (!picked.length) { pushLog('warn', '请至少选一个'); return {ok: false}; }
      return {ok: true, value: picked.slice()};
    },
  });
  return v;
}

/** 数字步进器：返回数字或 null */
async function uiNumber(title, desc, defVal = 1, min = 1, max = 99) {
  let val = clamp(Number(defVal) || min, min, max);
  const wrap = document.createElement('div');
  wrap.className = 'stepper';
  const minus = document.createElement('button');
  minus.textContent = '−';
  const valEl = document.createElement('span');
  valEl.className = 'val';
  const plus = document.createElement('button');
  plus.textContent = '+';
  const paint = () => { valEl.textContent = String(val); };
  minus.addEventListener('click', () => { val = clamp(val - 1, min, max); paint(); });
  plus.addEventListener('click', () => { val = clamp(val + 1, min, max); paint(); });
  paint();
  wrap.append(minus, valEl, plus);
  const v = await _uiShow({
    title, desc, body: wrap, okText: '确定',
    onOk: () => ({ok: true, value: val}),
  });
  return v;
}

// ══════════════════════════════════════════════════════════════
//  位姿编辑器（x / y / yaw 三件套）
//
//  ★ 为什么必须做这个东西（本轮两个 bug 的共同根）
//  ① 存点位没有 yaw：旧版 saveWaypoint 把 yaw 写死 0.0，界面上也从来不显示
//     朝向。结果是所有点位都"朝东站"，导航到位后车身朝哪全靠运气，而用户
//     在保存时完全无法察觉 —— 这不是一个能靠加个输入框解决的问题，必须
//     让朝向"看得见、点得动"。
//  ② 重新定位用不了：旧版 btnReloc 直接拿**当前位姿**当新位姿发出去，
//     等于"我在原地告诉 AMCL 我在原地"，除了把协方差重置以外什么都没做。
//     RViz 的 2D Pose Estimate 是"由人指定车实际在哪、朝哪"，所以这里必须
//     让用户能改 x/y，而不只是复述旧值。
//
//  所以两者共用同一个编辑器：坐标可手输、朝向用方向盘拖（也能手输弧度）、
//  附带 0/90/180/270 四个快捷方向。全部只靠点按，不强制打字。
// ══════════════════════════════════════════════════════════════

const RAD = (d) => (Number(d) || 0) * Math.PI / 180;      // 度 → 弧度
const DEG = (r) => (Number(r) || 0) * 180 / Math.PI;      // 弧度 → 度

/** 归一化到 (-π, π]，避免多次 +/-360° 后数值越滚越大 */
function normYaw(r) {
  let v = Number(r) || 0;
  while (v > Math.PI) v -= 2 * Math.PI;
  while (v <= -Math.PI) v += 2 * Math.PI;
  return v;
}

/** 通用位姿编辑对话框。
 *
 *  参数：
 *    init - {x, y, yaw} 初始值（必给，调用方负责取合理默认）
 *    opts - {
 *      title, desc,      标题 / 说明
 *      head,             插到位姿控件**上面**的内容（点位名输入之类，
 *                        放最上面是因为它是"这东西叫什么"，先定身份）
 *      tail,             插在朝向控件下面的内容（类型二选一之类）
 *      preview,          是否在地图上实时预览（默认 true）
 *      okText            确定按钮文字
 *    }
 *  返回：{x, y, yaw} 或 null（取消）
 *
 *  ★ 用选项对象而不是第 4/5/6 个位置参数：调用点有 3 处，参数一多
 *    `uiPose(p, '标题', '说明', null, true, '保存')` 这种就完全看不懂了。
 */
async function uiPose(init, opts) {
  const o = opts || {};
  const preview = o.preview !== false;
  const st = {
    x: Number(init.x) || 0,
    y: Number(init.y) || 0,
    yaw: normYaw(init.yaw),
  };

  const body = document.createElement('div');
  if (o.head) body.appendChild(o.head);

  // ── ① x / y 手输框 ──
  // ★ 用 inputmode=decimal 而不是 type=number：type=number 在手机上会
  //   出现上下箭头、并且部分安卓浏览器对 "-" 的输入处理很别扭；decimal
  //   只保证弹出数字键盘，输入框仍是普通文本，容错更好。
  const grid = document.createElement('div');
  grid.className = 'pose-grid';
  const mkNum = (label, key) => {
    const f = document.createElement('div');
    f.className = 'field';
    const lb = document.createElement('span');
    lb.textContent = label;
    const inp = document.createElement('input');
    inp.type = 'text';
    inp.inputMode = 'decimal';
    inp.value = fmt(st[key], 3);
    inp.setAttribute('aria-label', label);
    inp.addEventListener('input', () => {
      const v = parseFloat(inp.value);
      if (Number.isFinite(v)) { st[key] = v; syncPreview(); }
    });
    f.append(lb, inp);
    return {f, inp};
  };
  const fx = mkNum('x（米）', 'x');
  const fy = mkNum('y（米）', 'y');
  grid.append(fx.f, fy.f);
  body.appendChild(grid);

  // ── ② 朝向：方向盘 + 角度输入 + 快捷方向 ──
  const yawWrap = document.createElement('div');
  yawWrap.className = 'yaw-wrap';

  const dial = document.createElement('div');
  dial.className = 'yaw-dial';
  dial.title = '拖动或点击这个圆盘调整朝向';
  const arrowEl = document.createElement('div');
  arrowEl.className = 'arrow';
  const hub = document.createElement('div');
  hub.className = 'hub';
  dial.append(arrowEl, hub);

  const side = document.createElement('div');
  side.className = 'yaw-side';
  const yawField = document.createElement('div');
  yawField.className = 'field';
  const yawLabel = document.createElement('span');
  yawLabel.textContent = '朝向 yaw（度，0=朝东，逆时针为正）';
  const yawInp = document.createElement('input');
  yawInp.type = 'text';
  yawInp.inputMode = 'decimal';
  yawInp.setAttribute('aria-label', '朝向 yaw 角度');
  yawField.append(yawLabel, yawInp);

  const quick = document.createElement('div');
  quick.className = 'yaw-quick';
  const QUICKS = [['东', 0], ['北', 90], ['西', 180], ['南', -90]];
  const qbtns = QUICKS.map(([nm, deg]) => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = nm;
    b.title = `${nm} ${deg}°`;
    b.addEventListener('click', () => {
      st.yaw = normYaw(RAD(deg));
      syncUI();
      syncPreview();
    });
    quick.appendChild(b);
    return {b, deg};
  });

  side.append(yawField, quick);
  yawWrap.append(dial, side);
  body.appendChild(yawWrap);

  // 关键：把"方向盘的旋转、角度框的文字、快捷按钮的高亮"收在一个函数里，
  // 任何一处改动都调它。分散更新是这类控件最常见的 bug 源。
  const syncUI = () => {
    // 屏幕上：箭头默认指上 = 90°，所以要减 90°；再取负变成顺时针（CSS 正角）
    arrowEl.style.transform = `rotate(${-(DEG(st.yaw) - 90)}deg)`;
    // ★ 这里 st.yaw 存的是弧度，DEG() 之后就是度数，直接显示。
    //   （曾经写成 fmt(normYaw(RAD(d)), 1) —— d 已经是度了，再 RAD 一次
    //     就把 180° 显示成 3.1，用户看到的角度完全是错的。）
    if (document.activeElement !== yawInp) {
      yawInp.value = fmt(DEG(st.yaw), 1);
    }
    for (const {b, deg} of qbtns) {
      const want = normYaw(RAD(deg));
      b.classList.toggle('on', Math.abs(normYaw(st.yaw - want)) < 0.02);
    }
  };

  // 角度框输入 → 圆盘跟着转
  yawInp.addEventListener('input', () => {
    const v = parseFloat(yawInp.value);
    if (!Number.isFinite(v)) return;
    st.yaw = normYaw(RAD(v));
    arrowEl.style.transform = `rotate(${-(v - 90)}deg)`;
    for (const {b, deg} of qbtns) {
      b.classList.toggle('on', Math.abs(normYaw(st.yaw - RAD(deg))) < 0.02);
    }
    syncPreview();
  });

  // 方向盘：按下/拖动都按"从圆心指向手指"的方向来定 yaw。
  // ★ 用 pointer 事件 + setPointerCapture，这样手指滑出圆盘也不会断；
  //   用 touch 事件的话要么跟滚动打架，要么漏掉鼠标。
  const dialAt = (ev) => {
    const r = dial.getBoundingClientRect();
    const dx = ev.clientX - (r.left + r.width / 2);
    const dy = ev.clientY - (r.top + r.height / 2);
    if (Math.abs(dx) < 1 && Math.abs(dy) < 1) return;   // 正好点圆心：无意义
    st.yaw = normYaw(Math.atan2(-dy, dx));             // 屏幕 y 向下，故取负
    syncUI();
    syncPreview();
  };
  dial.addEventListener('pointerdown', (ev) => {
    ev.preventDefault();
    dial.setPointerCapture && dial.setPointerCapture(ev.pointerId);
    dialAt(ev);
  });
  dial.addEventListener('pointermove', (ev) => {
    if (!dial.hasPointerCapture || !dial.hasPointerCapture(ev.pointerId)) return;
    dialAt(ev);
  });

  if (o.tail) body.appendChild(o.tail);
  syncUI();

  // 地图上的实时预览：对话框开着时画一个黄标记，改一处动一处。
  // 取消/确定时都会清掉（见下面的 finally）。
  function syncPreview() {
    if (!preview) return;
    S.preview = {x: st.x, y: st.y, yaw: st.yaw};
    // 如果位置跑到视图外，把视图带过去，否则用户"改了坐标但看不见"
    const [sx, sy] = worldToScreen(st.x, st.y);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (sx < 0 || sy < 0 || sx > w || sy > h) {
      S.view.cx = st.x; S.view.cy = st.y;
      S.follow = false;
      const fb = $('btnFollow');
      if (fb) fb.classList.remove('on');
    }
    draw();
  }
  syncPreview();

  try {
    const v = await _uiShow({
      title: o.title, desc: o.desc, body, okText: o.okText || '确定',
      onOk: () => {
        // 以输入框里的最终文字为准再取一次，防止"刚敲完没触发 input"
        const px = parseFloat(fx.inp.value);
        const py = parseFloat(fy.inp.value);
        if (Number.isFinite(px)) st.x = px;
        if (Number.isFinite(py)) st.y = py;
        if (!Number.isFinite(st.x) || !Number.isFinite(st.y)) {
          pushLog('warn', '坐标必须是数字');
          return {ok: false};
        }
        return {ok: true, value: {x: st.x, y: st.y, yaw: normYaw(st.yaw)}};
      },
    });
    return v;
  } finally {
    // 无论确定还是取消，预览标记都要撤掉，否则地图上会留个黄箭头没人管
    S.preview = null;
    draw();
  }
}

// 模态框按钮与键盘绑定（只绑一次）
document.addEventListener('DOMContentLoaded', () => {
  const ok = $('modalOk');
  const cancel = $('modalCancel');
  if (ok) ok.addEventListener('click', () => {
    if (!_uiOnOk) return;
    const r = _uiOnOk();
    if (r && r.ok) _uiClose(r.value);
    // r.ok === false 表示校验没过，保持弹窗不动
  });
  if (cancel) cancel.addEventListener('click', () => _uiClose(null));
  const mask = $('modal');
  if (mask) mask.addEventListener('click', (e) => {
    if (e.target === mask) _uiClose(null);   // 点遮罩空白处 = 取消
  });
  document.addEventListener('keydown', (e) => {
    if ($('modal').classList.contains('hidden')) return;
    if (e.key === 'Escape') { _uiClose(null); }
    if (e.key === 'Enter') {
      e.preventDefault();
      if (_uiOnOk) { const r = _uiOnOk(); if (r && r.ok) _uiClose(r.value); }
    }
  });
});

// ─────────────── 日志渲染 ───────────────
function pushLog(level, text) {  S.logs.push({t: Date.now() / 1000, level, text});
  if (S.logs.length > 400) S.logs.splice(0, S.logs.length - 400);
  renderLog();
}

function renderLog() {
  const box = $('log');
  const atBottom = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
  const frag = document.createDocumentFragment();
  for (const it of S.logs.slice(-200)) {
    const d = document.createElement('div');
    d.className = 'ln';
    d.innerHTML = `<span class="tm">${nowStr(it.t)}</span>`
                + `<span class="tx lv-${it.level || 'info'}">${escapeHtml(it.text)}</span>`;
    frag.appendChild(d);
  }
  box.innerHTML = '';
  box.appendChild(frag);
  if (atBottom) box.scrollTop = box.scrollHeight;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) =>
    ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
}

// ══════════════════════════════════════════════════════════════
//  顶部实时仪表（固定 8 行）
//
//  ★ 为什么从"日志格子"改成"固定仪表"（用户原话）：
//    "数据面板显示的日志是固定死的，日志长的话看不到后面的信息。……
//     数据面板，就显示一些重点状态标志。"
//    上一版把日志原文塞进格子：避障一行 60~80 字，360px 侧栏一格只放
//    得下约 32 字，最该看的"减到多少"被截掉，而且没有任何展开入口。
//
//  ★ 现在的形态是**固定槽位**：8 行写死（任务/动作/速度/避障/定位/地图/
//    模式），值由后端 build_gauges 从状态量算出（不来自日志文本）。
//    好处：
//      1) 文本天生短（10~20 字），不存在"长了看不全"；
//      2) 位置固定，用户扫一眼就知道在哪看什么，不用每次找；
//      3) 7 个 DOM 节点建一次就永久复用，只改 textContent，
//         不增删、不重建、不闪、不动滚动位置。
// ══════════════════════════════════════════════════════════════

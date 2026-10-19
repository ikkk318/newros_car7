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

// 每个槽位初始占位。后端没数据时也要把 8 行画出来（骨架感），
// 而不是"面板空着"—— 空面板会让人以为功能没做。
const BOARD_SKELETON = [
  ['task', '任务'], ['act', '动作'], ['speed', '速度'],
  ['avoid', '避障'], ['pose', '定位'], ['map', '地图'],
  ['mode', '模式'], ['link', '连接'],
];

function clearBoard() {
  S.board = {};
  const box = $('board');
  if (box) box.innerHTML = '';
  const b = $('boardBadge');
  if (b) { b.textContent = '—'; b.className = 'badge unk'; }
}

// 建一行（只在第一次出现时调用；之后永远复用）
function makeBoardRow(key, label) {
  const el = document.createElement('div');
  el.id = 'bs-' + cssKey(key);
  el.className = 'brow';
  el.innerHTML = '<span class="bl"></span><span class="bv"></span>';
  el.querySelector('.bl').textContent = label;
  return el;
}

// 把一条仪表渲染/更新到 DOM。
function paintBoardSlot(slot) {
  const box = $('board');
  if (!box) return;
  let el = $('bs-' + cssKey(slot.key));
  if (!el) {
    el = makeBoardRow(slot.key, slot.label || slot.key);
    box.appendChild(el);
  }
  // ★ 关键：只有文字真的变了才写 DOM。
  //   textContent 赋值会触发重排，5 Hz × 8 行白白浪费手机性能；
  //   而且反复写同样的值会让屏幕上的数字"抖"（部分浏览器会重绘字形）。
  const val = el.querySelector('.bv');
  const text = (slot.text === undefined || slot.text === null
                || slot.text === '') ? '——' : String(slot.text);
  if (val.textContent !== text) val.textContent = text;
  // 色调 → class。class 变化不引起重排，只重绘，可以每帧设。
  const cls = 'brow tone-' + (slot.tone || 'off');
  if (el.className !== cls) el.className = cls;
}

// key 里可能有冒号、点、空格，不能直接当 id
function cssKey(k) {
  return String(k).replace(/[^a-zA-Z0-9_-]/g, '_');
}

function renderBoard(list) {
  const box = $('board');
  if (!box) return;
  // 后端没给数据（旧版本 / 拿不到快照）→ 按骨架画 8 行占位，
  // 而不是留一块空白。空白和"全都没数据"在视觉上必须可区分。
  const rows = (Array.isArray(list) && list.length) ? list
    : BOARD_SKELETON.map(([k, l]) => ({key: k, label: l, text: '——', tone: 'off'}));
  let warnN = 0;
  for (const slot of rows) {
    paintBoardSlot(slot);
    if (slot.tone === 'warn' || slot.tone === 'error') warnN++;
  }
  // 徽标：有异常就亮黄，否则灰。**不显示"共 N 条"** —— 固定 8 行，
  // 那个数字永远不变，纯占地方。
  const b = $('boardBadge');
  if (b) {
    b.textContent = warnN ? ('注意 ' + warnN) : '正常';
    b.className = 'badge ' + (warnN ? 'warn' : 'ok');
  }
}

// ══════════════════════════════════════════════════════════════
//  地图绘制
// ══════════════════════════════════════════════════════════════
const canvas = $('map');
const ctx = canvas.getContext('2d');

function resizeCanvas() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.round(r.width * dpr));
  canvas.height = Math.max(1, Math.round(r.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

// 世界坐标 → 屏幕像素
function worldToScreen(wx, wy) {
  const {cx, cy, scale} = S.view;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  return [ (wx - cx) * scale + w / 2, h / 2 - (wy - cy) * scale ];
}

// 屏幕像素 → 世界坐标
function screenToWorld(sx, sy) {
  const {cx, cy, scale} = S.view;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  return [ (sx - w / 2) / scale + cx, cy - (sy - h / 2) / scale ];
}

function draw() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);
  // 背景网格（便于判断尺度）
  drawGrid(w, h);

  const m = S.map.meta, img = S.map.img;
  if (img && m) {
    // 地图左下角与右上角在屏幕上的位置
    const [x0, y0] = worldToScreen(m.origin_x, m.origin_y);
    const mw = m.width * m.resolution * S.view.scale;
    const mh = m.height * m.resolution * S.view.scale;
    ctx.save();
    // origin 可能带 yaw：绕左下角旋转
    ctx.translate(x0, y0);
    if (m.origin_yaw) ctx.rotate(-m.origin_yaw);
    ctx.imageSmoothingEnabled = S.view.scale < 12;
    ctx.globalAlpha = 0.92;
    // ★ 图片从左上角画，而 (x0,y0) 是左下角，故 y 方向减去高度
    ctx.drawImage(img, 0, -mh, mw, mh);
    ctx.restore();
  }

  // 全局路径
  const plan = S.snap && S.snap.plan || [];
  if (plan.length > 1) {
    ctx.strokeStyle = '#52d1ff';
    ctx.lineWidth = 2.5;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    plan.forEach((p, i) => {
      const [sx, sy] = worldToScreen(p[0], p[1]);
      i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy);
    });
    ctx.stroke();
  }

  // 激光点（障碍物）
  const scan = S.snap && S.snap.scan || [];
  if (scan.length) {
    ctx.fillStyle = '#ffb454';
    const r = clamp(S.view.scale * 0.035, 1.2, 3);
    for (const p of scan) {
      const [sx, sy] = worldToScreen(p[0], p[1]);
      if (sx < -20 || sy < -20 || sx > w + 20 || sy > h + 20) continue;
      ctx.beginPath();
      ctx.arc(sx, sy, r, 0, 6.2832);
      ctx.fill();
    }
  }

  // 点位（可选显示）
  // ★ 点位是带朝向的（存了 yaw），但以前只画一个圆点 —— 用户完全看不出
  //   这个点位是"朝东等货"还是"朝西"，导航到位后车的朝向对不对也无从判断。
  //   这里给每个点位加一根短短的朝向线，一眼能看出站姿。缩得很小时线自然
  //   变短，不会糊成一团。
  if (S.waypoints && S.waypoints.length) {
    ctx.font = '11px sans-serif';
    // ★ 建图模式不画点位：那张图是新建的，点位属于别的坐标系，
    //   画上去就是"地图上没有的货架点"，纯干扰。
    if (S.mode !== 'mapping') {
      for (const wp of S.waypoints) {
        const [sx, sy] = worldToScreen(wp.x, wp.y);
        const col = wp.kind === 'cargo' ? '#c79bff' : '#4c9aff';
        ctx.fillStyle = col;
        ctx.beginPath();
        ctx.arc(sx, sy, 5, 0, 6.2832);
        ctx.fill();
        // 朝向线：屏幕 y 向下，故 y 分量取负
        const a = Number(wp.yaw) || 0;
        const len = clamp(S.view.scale * 0.22, 6, 22);
        ctx.strokeStyle = col;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        ctx.lineTo(sx + Math.cos(a) * len, sy - Math.sin(a) * len);
        ctx.stroke();
        ctx.fillStyle = '#e6eaf2';
        ctx.fillText(wp.name, sx + 8, sy + 4);
      }
    }
  }

  // 选中的位置（供存点位）+ 朝向
  // ★ yaw 也要画出来，否则"存点位没有 yaw"这个问题在界面上依然看不见：
  //   用户点了个位置、存下来是 yaw=0，但他没法在保存前知道/调整朝向。
  if (S.picked) drawPickedMark(S.picked, '#4c9aff', '#4c9aff');
  // 弹框里的预览标记（调朝向时实时跟着动）
  if (S.preview) drawPickedMark(S.preview, '#f0b429', 'rgba(240,180,41,.75)');

  // 小车（正方形车体 + 车头/车尾标注，见 drawRobot）
  const pose = S.snap && S.snap.pose;
  if (pose) drawRobot(pose);
}

function drawRobot(pose) {
  const [sx, sy] = worldToScreen(pose.x, pose.y);
  const scale = S.view.scale;
  const L = Math.max(12, 0.50 * scale);   // 车长 0.5 m
  const W = Math.max(12, 0.50 * scale);   // 车宽 0.5 m

  // ── 车体 ──
  // ★ 用户要求：不要三角形，改成正方形，并标出车头车尾。
  //   三角形在缩得很小的时候（地图缩到整屏），三个顶点几乎糊成一个点，
  //   朝向完全看不出来；正方形至少四条边清楚，再配一条车头色块 + 文字，
  //   远看近看都能判断车往哪边走。
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-pose.yaw);                  // 屏幕 y 向下，故取负；旋转后 +x = 车头
  ctx.beginPath();
  ctx.rect(-L / 2, -W / 2, L, W);
  ctx.fillStyle = 'rgba(62,207,142,.92)';
  ctx.fill();
  ctx.strokeStyle = '#0b1a14';
  ctx.lineWidth = 2;
  ctx.stroke();
  // 车头：+x 侧涂一条深色竖条（和"前"字一起看，色弱也分得清）
  const band = Math.max(3, L * 0.22);
  ctx.fillStyle = '#1d6b4c';
  ctx.fillRect(L / 2 - band, -W / 2, band, W);
  // 中轴指向线：从中心指向车头，拖出车体一小截
  ctx.strokeStyle = '#eafff5';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(L / 2 + 5, 0);
  ctx.stroke();
  ctx.restore();

  // ── 车头 / 车尾文字 ──
  // ★ 文字不跟着车体旋转：车斜着走时"车头"两个字要是也斜 45°，
  //   在手机上基本读不出来。所以用屏幕坐标自己算前/后两个落点，
  //   文字永远正着写。
  const ux = Math.cos(pose.yaw);          // 前向在屏幕上的 x 分量
  const uy = -Math.sin(pose.yaw);         // 前向在屏幕上的 y 分量（屏幕 y 向下）
  const off = L / 2 + 15;
  ctx.save();
  ctx.font = 'bold 10px sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.lineWidth = 3;
  // 先描一圈深色底，避免叠在地图白底上看不见
  ctx.strokeStyle = 'rgba(10,13,18,.85)';
  ctx.fillStyle = '#5ce6a8';
  ctx.strokeText('车头', sx + ux * off, sy + uy * off);
  ctx.fillText('车头', sx + ux * off, sy + uy * off);
  ctx.fillStyle = '#9aa7b8';
  ctx.strokeText('车尾', sx - ux * off, sy - uy * off);
  ctx.fillText('车尾', sx - ux * off, sy - uy * off);
  ctx.restore();
}

/** 画"选中位置"标记：十字 + 圆圈 + 朝向箭头。
 *
 *  参数：p - {x, y, yaw}；ring - 圆圈颜色；arrow - 朝向箭头颜色
 *  返回：无
 *
 *  ★ 为什么单独抽成函数：选中的位置和弹框里的预览要画成一样的样子，
 *    但颜色不同（蓝=已确定，黄=正在调）。复制两遍必然改一处漏一处。
 */
function drawPickedMark(p, ring, arrow) {
  const [sx, sy] = worldToScreen(p.x, p.y);
  ctx.strokeStyle = ring;
  ctx.lineWidth = 2;
  ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.arc(sx, sy, 12, 0, 6.2832);
  ctx.moveTo(sx - 18, sy); ctx.lineTo(sx + 18, sy);
  ctx.moveTo(sx, sy - 18); ctx.lineTo(sx, sy + 18);
  ctx.stroke();
  ctx.setLineDash([]);
  // 朝向：从圆心指出去的实心箭头（虚线圆 + 虚十字只表达"这里"，
  // 朝向必须有实体才看得清）
  const a = Number(p.yaw) || 0;
  const len = clamp(S.view.scale * 0.45, 16, 42);
  const ex = sx + Math.cos(a) * len;
  const ey = sy - Math.sin(a) * len;        // 屏幕 y 向下，故取负
  ctx.strokeStyle = arrow;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  // 箭头三角：屏幕坐标下的角度是 -a，尖端往回收两个 head
  const head = 7;
  const back = -a + Math.PI;
  ctx.fillStyle = arrow;
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex + Math.cos(back - 0.42) * head,
             ey + Math.sin(back - 0.42) * head);
  ctx.lineTo(ex + Math.cos(back + 0.42) * head,
             ey + Math.sin(back + 0.42) * head);
  ctx.closePath();
  ctx.fill();
}

function drawGrid(w, h) {
  const {scale} = S.view;
  // 选一个"好看"的格子间距：保证屏幕间距在 40~120 px
  let step = 1;
  const cand = [0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 50];
  for (const c of cand) { if (c * scale >= 45) { step = c; break; } step = c; }
  ctx.strokeStyle = 'rgba(255,255,255,.045)';
  ctx.lineWidth = 1;
  const [wx0, wy1] = screenToWorld(0, 0);
  const [wx1, wy0] = screenToWorld(w, h);
  const gx0 = Math.floor(wx0 / step) * step;
  const gy0 = Math.floor(wy0 / step) * step;
  ctx.beginPath();
  for (let x = gx0; x <= wx1; x += step) {
    const [sx] = worldToScreen(x, 0);
    ctx.moveTo(sx, 0); ctx.lineTo(sx, h);
  }
  for (let y = gy0; y <= wy1; y += step) {
    const [, sy] = worldToScreen(0, y);
    ctx.moveTo(0, sy); ctx.lineTo(w, sy);
  }
  ctx.stroke();
}

// 地图图片加载
//
// ★ 这里修的是"切模式后地图有重影"的两个直接原因：
//   ① 旧图残留在 img 里：换栈后新地图要等几秒才到，这几秒里 S.map.img
//      还是旧栈那张，而 meta 可能已经被新栈覆盖 → 旧像素按新几何画，
//      屏幕上就是两张图错位叠着。清理时机必须在**换栈事件**上，
//      不能等新图到了再替换。
//   ② 乱序响应：连续两次 loadMap()（例如 map_changed 连发两次），
//      浏览器可能让第 1 次请求后到，它带的是旧 PNG，直接覆盖新图 →
//      看起来像"切完还是旧地图"甚至"半新半旧"。
//      seq 是本地单调序号，只接受"最新的那次请求"，其余一律丢弃。
function loadMap(opts) {
  const force = !!(opts && opts.force);
  const seq = ++S.map.seq;
  const im = new Image();
  im.onload = () => {
    // 只认最后一次请求的结果；被后发请求超越的就是过期图，扔掉
    if (seq !== S.map.seq) return;
    S.map.img = im;
    draw();
  };
  im.onerror = () => {
    if (seq !== S.map.seq) return;
    if (force) pushLog('warn', '地图图片加载失败（地图可能尚未发布）');
  };
  // cache:'no-store' + 时间戳双保险：手机浏览器对 PNG 的启发式缓存很顽固，
  // 只加时间戳偶尔还会拿到缓存副本，重影就是这么来的。
  im.src = `/api/map.png?seq=${seq}&t=${Date.now()}`;
}

/** 换栈/换图时把"上一套栈的地图"彻底作废。
 *
 *  ★ 必须连 meta 一起清：只清 img 不清 meta，下次 draw() 时几何还在，
 *    网格和点位会按旧地图的 origin 画，看着像"地图没了但坐标还在飘"。
 *  参数：reason - 只用于日志
 */
function dropMapCache(reason) {
  S.map.img = null;
  S.map.meta = null;
  S.map.version = -1;
  S.map.seq += 1;            // 让所有在途的地图请求作废
  S.picked = null;           // 旧图上的选中点在新图上毫无意义
  S.preview = null;
  const pp = $('pickedPos');
  if (pp) {
    pp.textContent = '未选取位置';
    pp.classList.remove('set');
  }
  const ph = $('pickHint');
  if (ph) ph.textContent = '点地图取位置';
  if (reason) pushLog('info', `${reason}：已清掉上一张地图的显示缓存`);
  draw();
}

/** 后端说"地图代次变了"= 换了一整套栈，前端要做一次完整复位。
 *
 *  ★ 为什么不能只重载地图：换栈后 AMCL/SLAM 都是从零开始，旧位姿、
 *    旧激光点、旧全局路径全都属于另一套坐标系。只换图的话，小车会
 *    稳稳停在切换前的位置上，用户以为定位正常 —— 这比不显示更危险。
 */
function onMapEpoch(epoch) {
  if (epoch === S.map.epoch) return;
  const first = S.map.epoch < 0;
  S.map.epoch = epoch;
  if (first) return;                // 首次同步，不需要"复位"提示
  S.snap = null;                    // 旧位姿/激光/路径一律作废
  S.autoCenterDone = false;         // 重新自动居中（新图范围可能完全不同）
  S.follow = true;
  const fb = $('btnFollow');
  if (fb) fb.classList.add('on');
  dropMapCache('');
  pushLog('info', '检测到工作栈已切换：地图与小车位置显示已复位，'
    + '等待新栈发布地图/定位…');
  loadMap();
}

// 自动把视图对准小车 / 地图
function maybeCenter() {
  if (S.autoCenterDone) return;
  const pose = S.snap && S.snap.pose;
  const m = S.map.meta;
  if (pose) {
    S.view.cx = pose.x; S.view.cy = pose.y;
    S.autoCenterDone = true;
    draw();
  } else if (m) {
    S.view.cx = m.origin_x + m.width * m.resolution / 2;
    S.view.cy = m.origin_y + m.height * m.resolution / 2;
    // 自动缩放到地图能看见
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const fit = Math.min(w / (m.width * m.resolution),
                         h / (m.height * m.resolution)) * 0.9;
    if (fit > 0) S.view.scale = fit;
    S.autoCenterDone = true;
    draw();
  }
}

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
//  地图交互：单指拖动、双指缩放、长按设目标
// ══════════════════════════════════════════════════════════════
let drag = null, pinch = null, longPress = null;

function bindMap() {
  // ★★ 第一版这里有个很隐蔽的 bug，导致"点击地图选点没有任何反应"：
  //
  //      const hadLongPress = (longPress === null);   // 判"长按已触发"
  //
  //   而 longPress 只在 **触摸** 的 pointerdown 里才会被赋值：
  //      if (ev.pointerType === 'touch' && ev.isPrimary) { longPress = setTimeout(...) }
  //   鼠标点击走不进这个分支，longPress 永远是 null，于是
  //   hadLongPress 恒为 true → 后面 `!hadLongPress` 恒为 false →
  //   setPicked() 一次都不会被调用。电脑上点地图因此完全没反应。
  //
  //   现在改成用「按下到抬起的时长 + 位移」判断，鼠标和触摸走同一条逻辑，
  //   不再依赖 longPress 是否为 null 这种间接推断。
  const PICK_MAX_MS = 500;    // 按下超过这么久就算长按（设导航目标）
  const MOVE_PX = 6;          // 位移超过这么多像素就算拖动视图
  let down = null;            // {x, y, t, moved, longFired}

  canvas.addEventListener('pointerdown', (ev) => {
    const rect = canvas.getBoundingClientRect();
    const sx = ev.clientX - rect.left, sy = ev.clientY - rect.top;
    down = {x: ev.clientX, y: ev.clientY, t: Date.now(),
            moved: false, longFired: false};
    drag = {x: ev.clientX, y: ev.clientY, cx: S.view.cx, cy: S.view.cy};
    // 长按 = 设为导航目标（鼠标/触摸都支持，车间里用鼠标调试也很常见）
    if (longPress) clearTimeout(longPress);
    longPress = setTimeout(() => {
      longPress = null;
      if (!down || down.moved) return;
      down.longFired = true;
      const [wx, wy] = screenToWorld(sx, sy);
      setGoalAt(wx, wy);
    }, 650);
  });

  canvas.addEventListener('pointermove', (ev) => {
    if (!drag || !down) return;
    const dx = ev.clientX - down.x, dy = ev.clientY - down.y;
    if (Math.abs(dx) > MOVE_PX || Math.abs(dy) > MOVE_PX) {
      down.moved = true;
      if (longPress) { clearTimeout(longPress); longPress = null; }
      S.view.cx = drag.cx - dx / S.view.scale;
      S.view.cy = drag.cy + dy / S.view.scale;
      S.follow = false;
      $('btnFollow').classList.remove('on');
      draw();
    }
  });

  const endDrag = (ev) => {
    if (longPress) { clearTimeout(longPress); longPress = null; }
    // 单击/轻触（没拖动、没触发长按、抬起够快）= 在地图上选一个位置。
    // ★ 以前这里没判时长，鼠标稍微停一下就同时触发"设目标 + 选点"，
    //   车会突然动起来，用户根本不知道为什么。时长是必要条件。
    if (down && !down.moved && !down.longFired
        && (Date.now() - down.t) <= PICK_MAX_MS) {
      const rect = canvas.getBoundingClientRect();
      const cx = (ev && ev.clientX !== undefined) ? ev.clientX : down.x;
      const cy = (ev && ev.clientY !== undefined) ? ev.clientY : down.y;
      const [wx, wy] = screenToWorld(cx - rect.left, cy - rect.top);
      setPicked(wx, wy);
    }
    down = null;
    drag = null;
  };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', () => {
    if (longPress) { clearTimeout(longPress); longPress = null; }
    down = null;
    drag = null;
  });
  // ★ 鼠标按住拖到画布外再松开时 pointerup 不会落到 canvas 上，
  //   必须在这里把拖动状态收掉，否则回到画布后一动就跟着平移。
  //   注意：这里**不**调用 setPicked —— 选点只认 pointerup。
  canvas.addEventListener('pointerleave', () => {
    if (longPress) { clearTimeout(longPress); longPress = null; }
    down = null;
    drag = null;
  });

  // 滚轮缩放（电脑）
  canvas.addEventListener('wheel', (ev) => {
    ev.preventDefault();
    const f = ev.deltaY < 0 ? 1.12 : 1 / 1.12;
    S.view.scale = clamp(S.view.scale * f, 4, 400);
    draw();
  }, {passive: false});

  // 双指缩放（手机）
  canvas.addEventListener('touchstart', (ev) => {
    if (ev.touches.length === 2) {
      const [a, b] = ev.touches;
      pinch = {d: Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY),
               scale: S.view.scale};
    }
  }, {passive: true});
  canvas.addEventListener('touchmove', (ev) => {
    if (ev.touches.length === 2 && pinch) {
      ev.preventDefault();
      const [a, b] = ev.touches;
      const d = Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
      S.view.scale = clamp(pinch.scale * (d / pinch.d), 4, 400);
      draw();
    }
  }, {passive: false});
  canvas.addEventListener('touchend', () => { pinch = null; }, {passive: true});
}

async function setGoalAt(wx, wy) {
  try {
    await api('/api/command', {
      command: 'goto',
      args: [fmt(wx, 3), fmt(wy, 3), '0'],
    });
    pushLog('ok', `已在 (${fmt(wx)}, ${fmt(wy)}) 设为目标`);
  } catch (e) {
    pushLog('warn', `设目标失败：${e.message}`);
  }
}

// ══════════════════════════════════════════════════════════════
//  地图点选：单击=选位置（供存点位），长按=设导航目标
// ══════════════════════════════════════════════════════════════
function setPicked(wx, wy) {
  // ★ 单击地图时**不改已有朝向**：用户常常先点位置、再点"存为导航点"，
  //   在弹框里调好朝向；如果他手滑又点了一下地图，朝向不该被重置成 0。
  //   新选点的默认朝向取"小车当前朝向"（货架前停下时车头一般朝着货架，
  //   用它当默认值比 0 靠谱得多），拿不到位姿才退回 0。
  const pose = S.snap && S.snap.pose;
  S.picked = {
    x: wx, y: wy,
    yaw: S.picked ? S.picked.yaw : (pose ? pose.yaw : 0),
  };
  const el = $('pickedPos');
  el.textContent = `已选 x ${fmt(wx)}  y ${fmt(wy)}  θ ${fmt(DEG(S.picked.yaw), 0)}°`;
  el.classList.add('set');
  $('pickHint').textContent = '已选取';
  // ★ 必须立刻重绘一次：以前只改了文字，地图上的十字标记要等下一帧
  //   状态推送（5 Hz）才出现，看起来像是"点了没反应"。
  draw();
  pushLog('info', `已选取位置 x ${fmt(wx)}  y ${fmt(wy)}`
    + `（朝向 ${fmt(DEG(S.picked.yaw), 0)}°，可存为点位或重新定位）`);
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

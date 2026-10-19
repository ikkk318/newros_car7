/* ══════════════════════════════════════════════════════════════════
   map.js —— 地图画布：坐标变换、绘制、加载缓存、拖拽缩放、顶部仪表
   ══════════════════════════════════════════════════════════════════

   ★ 坐标约定（写错的话整张图会错位，看着很像"定位漂了"）
     ROS 世界坐标：x 向右、y 向上，单位米。
     Canvas 像素：x 向右、**y 向下**。
     地图 origin 是**左下角**在世界坐标里的位置。
     px = (x - origin_x)/res，py = height - (y - origin_y)/res

   ★ 本文件在顶层取 DOM（const canvas = $('map')），
     所以 index.html 里四个 <script> 必须放在 </body> 之前。
*/

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

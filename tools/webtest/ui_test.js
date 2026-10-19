/**
 * 前端交互测试（真跑 JS，不是静态检查）。
 *
 * ★ 为什么必须有它
 * 把 8 处 prompt() 换成自制模态框，属于"改了整条交互链路"：
 * 从同步阻塞（prompt 直接返回字符串）变成异步 Promise。
 * 只要漏一个 await，就会变成"点确定没反应"或者"undefined 被当点位名"，
 * 而这类错误 grep / 语法检查全都抓不到 —— 只有真的点一遍才知道。
 *
 * 这里用 jsdom 加载 index.html + app.js，把 fetch 打桩，
 * 然后模拟用户点按钮、点列表项、点确定，断言最终发给后端的 payload。
 */

const fs = require('fs');
const path = require('path');
const { JSDOM } = require('jsdom');

const ROOT = '/home/yuki/agv/newros_car7/src/agv_web/agv_web/static';
const html = fs.readFileSync(path.join(ROOT, 'index.html'), 'utf8');
const js = fs.readFileSync(path.join(ROOT, 'app.js'), 'utf8');
const css = fs.readFileSync(path.join(ROOT, 'app.css'), 'utf8');
// 后端源码也读一份：有些"前后端必须同时改"的约束要跨文件断言
const py = fs.readFileSync(path.join(ROOT, '..', 'web_gateway.py'), 'utf8');

let pass = 0;
let fail = 0;
const failures = [];

function ok(cond, name, extra) {
  if (cond) { pass++; console.log(`  ✓ ${name}`); }
  else {
    fail++;
    failures.push(name);
    console.log(`  ✗ ${name}${extra ? '  → ' + extra : ''}`);
  }
}

// ── 建立 DOM 环境 ──
// ★ runScripts:'dangerously' 才能让 app.js 的顶层函数声明挂到 window 上。
//   app.js 开头是 'use strict'，用 window.eval 执行时函数只在 eval 作用域内，
//   外部拿不到（会报 uiPrompt is not a function）。插成真实 script 标签，
//   script 顶层的 function 声明就是全局对象的属性，符合浏览器真实行为。
const dom = new JSDOM(html, {
  url: 'http://127.0.0.1:8080/?token=agv2024',
  runScripts: 'dangerously',
  pretendToBeVisual: true,
  beforeParse(w) {
    // canvas 打桩（jsdom 没有 2d 上下文），必须在解析脚本前装好
    w.HTMLCanvasElement.prototype.getContext = () => ({
      save() {}, restore() {}, translate() {}, rotate() {}, scale() {},
      clearRect() {}, fillRect() {}, strokeRect() {}, beginPath() {},
      moveTo() {}, lineTo() {}, closePath() {}, fill() {}, stroke() {},
      arc() {}, drawImage() {}, fillText() {}, setLineDash() {},
      rect() {}, ellipse() {}, strokeText() {},
      setTransform() {}, measureText: () => ({ width: 10 }),
      fillStyle: '', strokeStyle: '', lineWidth: 1, font: '',
      globalAlpha: 1, imageSmoothingEnabled: true, lineJoin: '', lineCap: '',
    });
    Object.defineProperty(w.HTMLElement.prototype, 'clientWidth',
                          { get() { return 800; }, configurable: true });
    Object.defineProperty(w.HTMLElement.prototype, 'clientHeight',
                          { get() { return 600; }, configurable: true });
    w.HTMLElement.prototype.getBoundingClientRect = function () {
      return { left: 0, top: 0, width: 800, height: 600,
               right: 800, bottom: 600, x: 0, y: 0 };
    };
  },
});
const { window } = dom;
const { document } = window;

// ── fetch 打桩：记录每次请求 ──
const calls = [];
// 假后端的可变状态（模拟"任务在跑 / 停了"这种会变的量）
const STATE = { taskRunningStub: false };
const ROUTE = {
  '/api/state': { ok: true, pose: { x: 0, y: 0, yaw: 0 }, scan: [], plan: [],
                  map_version: 0, map_meta: null, mode: 'nav',
                  lease: { owner: null, free: true } },
  '/api/logs': { logs: [] },
  '/api/maps': { ok: true, maps: ['world666map', '厂房A'], current: '' },
  '/api/stack': { ok: true, stack: { mode: '', running: false, manage: true,
                                     mode_text: '未启动', teleop_topic: '/agv/web_teleop' } },
  '/api/store': (body) => {
    if (body.action === 'list') {
      return { ok: true, names: ['货架A', '充电桩', '库位1'], items: [], count: 3 };
    }
    if (body.action === 'get') {
      return { ok: true, items: [{ name: body.name, x: 1, y: 1, yaw: 0, kind: 'cargo' }] };
    }
    return { ok: true, message: '已保存点位「' + body.name + '」', names: [], items: [] };
  },
  '/api/routes': (body) => {
    if (body.action === 'list') return { ok: true, names: [], counts: [] };
    if (body.action === 'save') {
      return { ok: true, message: `已保存路线「${body.name}」，共 ${body.point_names.length} 个途经点`, names: [body.name], counts: [body.point_names.length] };
    }
    return { ok: true, message: 'ok', items: [] };
  },
  '/api/tasks': (body) => {
    // ★ 这个桩要模拟**后端真实语义**，否则测不出"中止后能不能再启动"。
    //   以前所有分支都返回 running:false，等于假装永远没在跑，
    //   start 之后立刻查又是 false —— 那种桩让 bug 藏得住。
    const a = (body && body.action) || 'list';
    if (a === 'add') {
      return { ok: true, tasks: [{ pick: body.pick, drop: body.drop, loops: body.loops }], running: false };
    }
    if (a === 'clear') return { ok: true, tasks: [] };
    if (a === 'start') {
      STATE.taskRunningStub = true;
      return { ok: true, message: '已开始执行任务', running: true };
    }
    if (a === 'stop') {
      const was = STATE.taskRunningStub;
      STATE.taskRunningStub = false;
      return { ok: true, running: false, message: was ? '已中止任务' : '当前没有正在执行的任务' };
    }
    // list / 缺省
    return { ok: true, tasks: [], running: !!STATE.taskRunningStub };
  },
  '/api/stack_start': () => ({ ok: true, message: '启动中', stack: { mode: 'nav', running: true } }),
  '/api/teleop': { ok: true },
  '/api/lease': { ok: true, owner: 'me' },
  '/api/check': { ok: true, items: [] },
  '/api/map_save': { ok: true, message: '地图已保存' },
  '/api/map_delete': { ok: true, message: '已删除' },
  '/api/command': { ok: true },
  '/api/cargo': { ok: true, message: 'ok' },
  '/api/shutdown': { ok: true, message: '正在停止并退出…' },
};

window.fetch = async (url, opt) => {
  const u = new URL(url, 'http://127.0.0.1:8080');
  let body = undefined;
  if (opt && opt.body) { try { body = JSON.parse(opt.body); } catch (_) {} }
  const p = u.pathname;
  calls.push({ path: p, body, search: u.search });
  let data = ROUTE[p];
  if (typeof data === 'function') data = data(body);
  if (data === undefined) data = { ok: true };
  return {
    ok: true, status: 200,
    json: async () => data,
  };
};

// ── 执行 app.js（插成真实 script 标签，函数才会挂到 window）──
try {
  const el = document.createElement('script');
  el.textContent = js;
  document.body.appendChild(el);
} catch (e) {
  console.log('  ✗ app.js 执行抛异常: ' + e.message);
  process.exit(1);
}
ok(typeof window.uiPrompt === 'function', 'app.js 执行后 uiPrompt 可用');
ok(typeof window.uiPick === 'function', 'uiPick 可用');
ok(typeof window.uiMulti === 'function', 'uiMulti 可用');
ok(typeof window.uiNumber === 'function', 'uiNumber 可用');
ok(typeof window.saveWaypoint === 'function', 'saveWaypoint 可用');
ok(typeof window.newRoute === 'function', 'newRoute 可用');
ok(typeof window.addTask === 'function', 'addTask 可用');
ok(typeof window.startStack === 'function', 'startStack 可用');
ok(typeof window.saveMap === 'function', 'saveMap 可用');

// 触发 DOMContentLoaded（app.js 里的渲染绑定挂在这上面）
document.dispatchEvent(new window.Event('DOMContentLoaded'));
ok(true, 'DOMContentLoaded 处理完成');

// ── 工具：等 DOM 更新 ──
const tick = (ms = 20) => new Promise((r) => setTimeout(r, ms));
const $ = (id) => document.getElementById(id);
const modalOpen = () => !$('modal').classList.contains('hidden');

function clickModalOk() { $('modalOk').click(); }
function clickModalCancel() { $('modalCancel').click(); }

(async () => {
  console.log('\n【1】模态框基础设施');
  ok(!!$('modal'), '页面有 #modal 容器');
  ok($('modal').classList.contains('hidden'), '初始时模态框是隐藏的');

  const p1 = window.uiPrompt('测试', '说明', '默认值');
  await tick();
  ok(modalOpen(), 'uiPrompt 打开模态框');
  ok($('modalTitle').textContent === '测试', '标题正确');
  ok($('modalBody').querySelector('input').value === '默认值', '输入框预填默认值');
  clickModalOk();
  const v1 = await p1;
  await tick();
  ok(v1 === '默认值', `直接点确定返回默认值（实际 ${JSON.stringify(v1)}）`);
  ok(!modalOpen(), '确定后模态框关闭');

  console.log('\n【2】取消/遮罩/Esc 行为');
  const p2 = window.uiPrompt('测试2', '', 'x');
  await tick();
  clickModalCancel();
  ok((await p2) === null, '点取消返回 null');
  const p3 = window.uiConfirm('确认', '要不要', '是');
  await tick();
  ok(modalOpen(), 'uiConfirm 打开');
  clickModalOk();
  ok((await p3) === true, 'uiConfirm 确定返回 true');

  console.log('\n【3】单选列表 uiPick');
  const p4 = window.uiPick('选地图', '挑一个', [
    { value: 'world666map', label: 'world666map' },
    { value: '厂房A', label: '厂房A' },
  ], '厂房A');
  await tick();
  ok(modalOpen(), 'uiPick 打开');
  const opts = $('modalBody').querySelectorAll('.opt');
  ok(opts.length === 2, `列表渲染 2 项（实际 ${opts.length}）`);
  ok($('modalBody').querySelectorAll('.opt.on').length === 1, '默认选中 1 项');
  opts[0].click();          // 改成选第一项
  await tick();
  ok($('modalBody').querySelectorAll('.opt.on')[0].textContent.includes('world666map'),
     '点击后选中项切换为第一项');
  clickModalOk();
  ok((await p4) === 'world666map', '返回点击选中的值');

  console.log('\n【4】多选带顺序 uiMulti（替代"逗号分隔输入"）');
  const p5 = window.uiMulti('选途经点', '按顺序点', [
    { value: 'A', label: 'A' }, { value: 'B', label: 'B' },
    { value: 'C', label: 'C' },
  ]);
  await tick();
  let os = $('modalBody').querySelectorAll('.opt');
  os[2].click(); await tick();          // 先点 C
  os = $('modalBody').querySelectorAll('.opt');
  os[0].click(); await tick();          // 再点 A
  os = $('modalBody').querySelectorAll('.opt');
  const ords = [...$('modalBody').querySelectorAll('.ord')].map((e) => e.textContent);
  ok(ords.join(',') === '2,1' || ords.join(',') === '1,2',
     `选中项显示顺序编号（实际 ${ords.join(',')}）`);
  clickModalOk();
  const picked = await p5;
  ok(JSON.stringify(picked) === '["C","A"]',
     `返回值为点击顺序 ["C","A"]（实际 ${JSON.stringify(picked)}）`);

  // 再点一次取消
  const p6 = window.uiMulti('多选', '', [
    { value: 'X', label: 'X' }, { value: 'Y', label: 'Y' },
  ]);
  await tick();
  $('modalBody').querySelectorAll('.opt')[0].click(); await tick();
  $('modalBody').querySelectorAll('.opt')[0].click(); await tick();
  ok($('modalBody').querySelectorAll('.opt.on').length === 0, '再点一次可取消选中');
  clickModalOk();   // 空选时应被拦下
  await tick();
  ok(modalOpen(), '空选时点确定不关闭（校验拦住）');
  clickModalCancel();
  ok((await p6) === null, '取消返回 null');

  console.log('\n【5】数字步进器 uiNumber');
  const p7 = window.uiNumber('次数', '', 1, 1, 99);
  await tick();
  const btns = $('modalBody').querySelectorAll('.stepper button');
  btns[1].click(); btns[1].click();     // +2
  await tick();
  ok($('modalBody').querySelector('.stepper .val').textContent === '3', '步进器 + 两次变成 3');
  btns[0].click(); await tick();
  ok($('modalBody').querySelector('.stepper .val').textContent === '2', '步进器 - 一次变成 2');
  clickModalOk();
  ok((await p7) === 2, '返回数字 2');

  console.log('\n【6】业务流程：存点位（原 prompt 场景）');
  calls.length = 0;
  // 直接调用 saveWaypoint：先造一个"已选位置"
  window.eval('S.picked = {x: 1.23, y: 4.56};');
  const sp = window.saveWaypoint('cargo');
  await tick();
  ok(modalOpen(), '存点位弹出模态框（不再是系统 prompt）');
  const defName = $('modalBody').querySelector('input').value;
  ok(/^货架\d+$/.test(defName), `默认名按规则生成（实际 ${defName}）`);
  clickModalOk();
  await sp;
  await tick();
  const saveCall = calls.find((c) => c.path === '/api/store' && c.body
                                    && c.body.action === 'save');
  ok(!!saveCall, '确实调用了 /api/store save');
  ok(saveCall && saveCall.body.name === defName, '提交的名字 = 默认名');
  ok(saveCall && saveCall.body.x === 1.23 && saveCall.body.y === 4.56,
     '提交的坐标来自地图点选');
  ok(saveCall && saveCall.body.kind === 'cargo', 'kind=cargo 正确');

  console.log('\n【7】业务流程：新建路线（原"逗号分隔输入"场景）');
  calls.length = 0;
  // 先让 S.waypoints 有点位
  window.eval("S.waypoints = [{name:'货架A',x:0,y:0},{name:'充电桩',x:1,y:1},{name:'库位1',x:2,y:2}];");
  const nr = window.newRoute();
  await tick();
  ok(modalOpen(), '第一步：路线名模态框');
  const rname = $('modalBody').querySelector('input').value;
  ok(/^路线\d+$/.test(rname), `路线名有默认值（实际 ${rname}）`);
  clickModalOk();
  await tick(40);
  ok(modalOpen(), '第二步：选途经点的模态框已打开');
  const ropts = $('modalBody').querySelectorAll('.opt');
  ok(ropts.length === 3, `途经点列表 3 项（实际 ${ropts.length}）`);
  ropts[1].click(); await tick();
  $('modalBody').querySelectorAll('.opt')[2].click(); await tick();
  clickModalOk();
  await nr;
  await tick();
  const routeSave = calls.find((c) => c.path === '/api/routes' && c.body
                                     && c.body.action === 'save');
  ok(!!routeSave, '调用了 /api/routes save');
  ok(routeSave && JSON.stringify(routeSave.body.point_names) === '["充电桩","库位1"]',
     `途经点 = 点击顺序，无需输入逗号（实际 ${JSON.stringify(routeSave && routeSave.body.point_names)}）`);

  console.log('\n【8】业务流程：新增任务（原有 3 个 prompt → 现在是 4 步点选）');
  calls.length = 0;
  window.eval("S.routes = ['1号','路线1']; S.routeCounts = [4,2];");
  const at = window.addTask();
  await tick();
  ok(modalOpen(), '第 1 步：选取货点');
  $('modalBody').querySelectorAll('.opt')[0].click(); await tick();
  clickModalOk();
  await tick(40);
  ok(modalOpen(), '第 2 步：选放货点');
  const dOpts = $('modalBody').querySelectorAll('.opt');
  dOpts[1].click(); await tick();
  clickModalOk();
  await tick(40);
  // ★ 第 3 步是新增的"选路线"。以前网页根本没法给任务指定路线，
  //   而后端 /api/tasks add 的 route 字段一直存在 —— 功能存着没入口。
  ok(modalOpen(), '第 3 步：选路线（本轮新增）');
  const rOpts = $('modalBody').querySelectorAll('.opt');
  ok(rOpts.length === 3, `路线列表 = "不走路线" + 2 条（实际 ${rOpts.length}）`);
  ok((rOpts[0].textContent || '').indexOf('不走路线') >= 0,
     '第一条是"不走路线"，默认直接过');
  rOpts[1].click(); await tick();        // 选「1号」
  clickModalOk();
  await tick(40);
  ok(modalOpen(), '第 4 步：循环次数');
  ok(!!$('modalBody').querySelector('.stepper'), '循环次数是步进器而不是输入框');
  $('modalBody').querySelectorAll('.stepper button')[1].click(); await tick();
  clickModalOk();
  await at;
  await tick();
  const taskAdd = calls.find((c) => c.path === '/api/tasks' && c.body
                                   && c.body.action === 'add');
  ok(!!taskAdd, '调用了 /api/tasks add');
  ok(taskAdd && taskAdd.body.pick === '货架A' && taskAdd.body.drop === '充电桩',
     `取放点正确（${taskAdd && taskAdd.body.pick} ⇒ ${taskAdd && taskAdd.body.drop}）`);
  ok(taskAdd && taskAdd.body.loops === 2, `循环次数=2（实际 ${taskAdd && taskAdd.body.loops}）`);
  ok(taskAdd && taskAdd.body.route === '1号',
     `路线随任务一起提交（实际 ${JSON.stringify(taskAdd && taskAdd.body.route)}）`);

  console.log('\n【8b】任务的路线要显示在列表里、且能改');
  calls.length = 0;
  window.eval("S.tasks = [{pick:'A',drop:'B',loops:1,route:'1号'},"
            + "{pick:'C',drop:'D',loops:2,route:''}]; S.taskRunning=false;");
  window.eval('renderTasks()');
  const rows = $('taskList').querySelectorAll('.wp');
  ok(rows.length === 2, `列出 2 条任务（实际 ${rows.length}）`);
  ok((rows[0].textContent || '').indexOf('1号') >= 0,
     '带路线的任务要显示路线名（否则用户不知道走不走路线）');
  ok((rows[1].textContent || '').indexOf('直连') >= 0,
     '不带路线的任务显示"直连"');
  const er = window.editTaskRoute(0);
  await tick();
  ok(modalOpen(), '点任务行弹出改路线框');
  const eo = $('modalBody').querySelectorAll('.opt');
  ok(eo.length === 3, `改路线列表 3 项（实际 ${eo.length}）`);
  eo[2].click(); await tick();           // 改成「路线1」
  clickModalOk();
  await er;
  await tick();
  const rep = calls.find((c) => c.path === '/api/tasks' && c.body
                              && c.body.action === 'replace');
  ok(!!rep, '走 replace 整表覆盖（保顺序，不用删了重建）');
  ok(rep && rep.body.tasks[0].route === '路线1' && rep.body.tasks[1].route === '',
     `只改这一条的路线，其它不动（实际 ${JSON.stringify(rep && rep.body.tasks)}）`);

  console.log('\n【9】业务流程：启动导航选地图（原 prompt）');
  calls.length = 0;
  window.eval("S.maps = ['world666map','厂房A']; S.currentMap='';");
  const ss = window.startStack('nav');
  await tick();
  ok(modalOpen(), '多地图时弹出点选地图');
  ok($('modalBody').querySelectorAll('.opt').length === 2, '列出 2 张地图');
  $('modalBody').querySelectorAll('.opt')[1].click(); await tick();
  clickModalOk();
  await ss;
  await tick();
  const stCall = calls.find((c) => c.path === '/api/stack_start');
  ok(!!stCall, '调用了 /api/stack_start');
  ok(stCall && stCall.body.map === '厂房A', `选中地图传对了（${stCall && stCall.body.map}）`);

  console.log('\n【10】保存地图默认名');
  calls.length = 0;
  const sm = window.saveMap();
  await tick();
  const mName = $('modalBody').querySelector('input').value;
  ok(/^地图\d+月\d+日\d+时$/.test(mName), `默认地图名带日期（实际 ${mName}）`);
  clickModalOk();
  await sm;
  await tick();
  const msCall = calls.find((c) => c.path === '/api/map_save');
  ok(msCall && msCall.body.name === mName, '保存地图用的默认名');

  console.log('\n【12】导航不需要先建图：有地图就弹选图确认');
  // ★ 用户核心抱怨："导航模式怎么还要建图才能导航，我不是已经放好地图了吗"
  //   根因是 POST /api/maps 吃 405 → maps 变空 → 前端以为没图。
  //   这里验证：地图库只有一张图时**仍然弹选图框**（用户明确要求
  //   "每次启动都要弹一次，哪怕只有一张"，理由是防误点「启动导航」）。
  //
  //   注意不能直接改 window.S.maps —— S 在 app.js 里是 `const`，
  //   不是 window 的属性。只能通过公开函数走真实路径：
  //   改假的 /api/maps 返回值 → 调 refreshMaps() → 再 startStack()。
  calls.length = 0;
  const _mapsBackup = ROUTE['/api/maps'];
  ROUTE['/api/maps'] = {
    ok: true, maps: ['world666map'], current: '',
    info: { world666map: { points: 15, saved_at: '2026-10-17 22:10', has_routes: true } },
  };
  await window.refreshMaps();
  await tick();
  const navP = window.startStack('nav');
  await tick();
  ok(!$('modal').classList.contains('hidden'), '只有一张图也要弹选图框（不是自动开）');
  ok(/选择导航地图/.test($('modalTitle').textContent),
     `弹的是选地图框（${$('modalTitle').textContent}）`);
  const oneOpts = $('modalBody').querySelectorAll('.opt');
  ok(oneOpts.length === 1, `列出了 1 张图（实际 ${oneOpts.length}）`);
  // 副标题要显示"几个点位"，地图多了才分得清
  ok(/15 个点位/.test($('modalBody').textContent), '副标题显示点位数');
  // ★ 关掉弹框前不能发启动请求：这就是加确认框的意义
  ok(calls.filter((c) => c.path === '/api/stack_start').length === 0,
     '未确认时绝不启动（防误点拉栈）');
  clickModalOk();
  await navP;
  await tick();
  const navCall = calls.find((c) => c.path === '/api/stack_start');
  ok(!!navCall, '确认后调用了 /api/stack_start');
  ok(navCall && navCall.body.mode === 'nav', '模式是 nav');
  ok(navCall && navCall.body.map === 'world666map', `地图传对（${navCall && navCall.body.map}）`);

  console.log('\n【13】地图库真的空才提示建图');
  calls.length = 0;
  ROUTE['/api/maps'] = { ok: true, maps: [], current: '' };
  await window.refreshMaps();
  await tick();
  await window.startStack('nav');
  await tick();
  const startCallsWhenEmpty = calls.filter((c) => c.path === '/api/stack_start');
  ok(startCallsWhenEmpty.length === 0, '没图时不发启动请求（避免起个空栈）');
  // ★ 关键：本地为空时应重问一次后端才下结论，而不是一次失败就判死
  const mapTries = calls.filter((c) => c.path === '/api/maps').length;
  ok(mapTries >= 2, `本地为空时会重问后端（实际问了 ${mapTries} 次）`);
  ROUTE['/api/maps'] = _mapsBackup;
  await window.refreshMaps();

  console.log('\n【12b】选图取消 = 什么都不发生');
  // ★ 加确认框的唯一目的就是"给一次反悔的机会"。
  //   如果取消之后还是把栈拉起来了，这个框反而更糟（用户以为取消了）。
  calls.length = 0;
  const _mapsBackup2 = ROUTE['/api/maps'];
  ROUTE['/api/maps'] = { ok: true, maps: ['world666map'], current: '' };
  await window.refreshMaps();
  await tick();
  const cancelP = window.startStack('nav');
  await tick();
  ok(!$('modal').classList.contains('hidden'), '弹了选图框');
  clickModalCancel();
  const cancelResult = await cancelP;
  await tick();
  ok($('modal').classList.contains('hidden'), '取消后弹框关闭');
  ok(calls.filter((c) => c.path === '/api/stack_start').length === 0,
     '取消后没有启动栈（这是确认框的全部意义）');
  ok(cancelResult === undefined || cancelResult === null,
     '取消时函数正常返回，不抛异常');
  ROUTE['/api/maps'] = _mapsBackup2;
  await window.refreshMaps();

  console.log('\n【14】退出：停栈 + 关网关');
  calls.length = 0;
  const quitP = window.shutdownAll();
  await tick();
  const quitTitle = $('modalTitle').textContent;
  ok(/退出/.test(quitTitle), `弹出确认框（${quitTitle}）`);
  clickModalOk();
  await quitP;
  await tick();
  const sdCall = calls.find((c) => c.path === '/api/shutdown');
  ok(!!sdCall, '调用了 /api/shutdown');
  ok(sdCall && sdCall.body.stop_stack === true, '明确要求停栈');
  ok(/已退出/.test($('stackHint').textContent), '界面给出已退出状态');

  console.log('\n【15】控制权续期与丢失提示');
  // 前端必须在"持有控制权"时定时续期，否则用户思考几秒锁就掉了
  ok(/action: 'renew'/.test(js), '前端有 renew 调用');
  ok(/setInterval\([\s\S]{0,300}renew/.test(js), 'renew 有定时器驱动');
  ok(/visibilitychange/.test(js), '页面重新可见时补续期');
  ok(/控制权已被/.test(js), '丢失控制权时给明确提示（不静默）');
  ok(/beforeunload/.test(js) && /sendBeacon/.test(js), '离开页面自动交还控制权');
  // 刷新页面不能停栈：beforeunload 里只许 release，不许 shutdown
  const bl = (js.match(/beforeunload[\s\S]{0,700}/) || [''])[0];
  ok(!/api\/shutdown/.test(bl), '刷新页面不能触发关机（beforeunload 里不含 shutdown）');
  // 后端超时也要足够宽松，否则前端续期再勤也没用
  ok(/lease_idle_sec/.test(py), '后端仍可配置 lease_idle_sec');

  console.log('\n【16】任务中止后能再启动（用户报的"卡在执行中"）');
  // ★ 用户原话："执行完任务后还是显示执行中，且手动点击中断任务后也依旧
  //   显示执行中，导致之后不能终止任务也不能启动新任务。"
  //   根因有三层，这里逐层验证前端这一层：
  //     ① /api/tasks 的 POST 没有 list 分支 → refreshTasks 永远 400
  //        → S.taskRunning 不更新 → 按钮永远"执行中…"
  //     ② 中止只发了裸 cancel_task，没有置后端的停止事件
  //     ③ 前端 chipState 直显英文状态码
  calls.length = 0;
  STATE.taskRunningStub = false;

  // 16.1 启动
  await window.runTasks();
  await tick();
  ok(STATE.taskRunningStub === true, '后端进入执行状态');
  ok($('btnTaskRun').textContent.includes('执行中'), '按钮显示执行中');

  // 16.2 中止 —— 走 _uiShow 确认框
  const stopP = window.runTasks();
  await tick();
  ok(/中止/.test($('modalTitle').textContent), '弹出中止确认框');
  clickModalOk();
  await stopP;
  await tick();
  const stopCall = calls.find((c) => c.path === '/api/tasks' && c.body.action === 'stop');
  ok(!!stopCall, '中止走 /api/tasks 的 stop 动作（不是裸 cancel_task）');
  ok(!calls.some((c) => c.path === '/api/command' && c.body.command === 'cancel_task'),
     '不再用裸 cancel_task 命令中止');
  ok($('btnTaskRun').textContent.includes('开始执行'),
     `中止后按钮立刻变回"开始执行"（实际 ${$('btnTaskRun').textContent}）`);

  // 16.3 中止后能再启动 —— 这就是用户"不能启动新任务"的场景
  calls.length = 0;
  await window.runTasks();
  await tick();
  ok(STATE.taskRunningStub === true, '中止后能再次启动');
  ok($('btnTaskRun').textContent.includes('执行中'), '再次进入执行中');
  // 收尾，别把状态留给后面的测试
  ROUTE['/api/tasks']({action: 'stop'});
  STATE.taskRunningStub = false;

  console.log('\n【17】任务状态码显示中文（不再是一堆英文）');
  ok(/TASK_STATE_ZH/.test(js), '有状态码→中文的映射表');
  for (const [code, zh] of Object.entries({
    IDLE: '空闲', GO_LOAD: '取货中', LIFTING: '顶升中',
    TRANSPORT: '运输中', LOWERING: '放货中', DONE: '已完成',
    ERROR: '异常', PAUSED: '已暂停',
  })) {
    ok(new RegExp(code + "\\s*:\\s*'" + zh + "'").test(js),
       `状态 ${code} → ${zh}`);
  }
  // DONE 必须判成"成功"色，不能落进错误分支
  ok(!/DONE/.test(js.match(/if \(s === 'DONE'\) return '(\w+)'/)?.[1] || ''),
     'DONE 不判成错误色');
  ok(/if \(s === 'DONE'\) return 'ok'/.test(js), "DONE 显式返回 'ok'");

  console.log('\n【18】日志降噪：连接断开不再刷屏');
  // ★ 用户吐槽："总是一直打印连接断开"
  ok(!/pushLog\('warn', '连接断开，2 秒后重试'\)/.test(js),
     '删掉了"每 2 秒刷一条连接断开"');
  ok(/_wsFails/.test(js), '有重试计数，用于抑制重复提示');
  ok(/_warnOnce/.test(js), '有"同类警告只提示一次"的机制');
  ok(/与小车连接断开，正在自动重连/.test(js), '首次断线仍会提示一次');
  ok(/连接持续失败/.test(js), '连续失败多次才升级为错误提示');
  // ★ 真车实测：手机 WiFi 每 10~26 秒抖一次，2~3 秒就重连回来。
  //   原来"每次断线都 pushLog"（onopen 把 _wsFails 清零 → 下次又算第一次）
  //   让"断开/连上"成对刷了十几遍，业务日志全被冲走。现在必须等
  //   **断开超过 5 秒**才提示，短抖静默。
  ok(/_wsDownTimer = setTimeout/.test(js) && /}, 5000\)/.test(js),
     '★ 断线提示有 5 秒延时门限（短抖不写日志）');
  ok(/if \(S\._wsWasDown && S\._wsDownLogged\)/.test(js),
     '★ 只有提示过断开才补"重新连上"（避免成对刷屏）');
  // 退避重试：不能死板地每 2 秒打一次
  ok(/Math\.min\(2000 \* S\._wsFails, 10000\)/.test(js),
     '重试间隔逐步拉长（2s→10s），避免疯狂打网关');

  console.log('\n【11】CSS 覆盖检查');
  ok(/\.modal-mask/.test(css), 'CSS 有 .modal-mask');
  ok(/\.opt\.on/.test(css), 'CSS 有选中态 .opt.on');
  ok(/min-height:48px/.test(css), '可点元素 ≥48px（触屏友好）');
  ok(/font-size:16px/.test(css), '输入框 16px（防 iOS 自动放大）');
  ok(/env\(safe-area-inset-bottom\)/.test(css), '适配刘海屏安全区');

  console.log('\n【19】地图点选：单击地图必须真的能选到位置');
  // ★ 用户报的 bug："点击地图选点没有任何反应"。
  //   根因是 bindMap 的 endDrag 里用了 `longPress === null` 来当
  //   "长按已触发"的判据，而 longPress 只在触摸 pointerdown 里赋值 ——
  //   鼠标点击时它恒为 null → hadLongPress 恒 true → setPicked 永不调用。
  //   这类 bug 静态检查完全抓不到，必须真的派发事件点一下。
  //
  //   注意：canvas 的交互绑定在 boot() 里（登录成功后），本测试没走登录，
  //   所以要显式调一次 bindMap()，否则监听器一个都没挂，测了个寂寞。
  window.eval('bindMap()');
  window.eval("S.picked = null; S.view.cx = 0; S.view.cy = 0; S.view.scale = 40;");
  const cv = $('map');
  const pd = (type, x, y) => cv.dispatchEvent(new window.MouseEvent(type, {
    clientX: x, clientY: y, bubbles: true, cancelable: true, view: window,
  }));
  pd('pointerdown', 400, 300);
  await tick(10);
  pd('pointerup', 400, 300);
  await tick(10);
  ok(window.eval('S.picked') !== null,
     '鼠标单击地图就选到了位置（以前完全没反应）');
  const pk = window.eval('S.picked');
  ok(pk && Math.abs(pk.x) < 0.001 && Math.abs(pk.y) < 0.001,
     `屏幕中心对应世界原点（实际 ${JSON.stringify(pk)}）`);
  ok(($('pickedPos').textContent || '').indexOf('已选') >= 0,
     '#pickedPos 显示"已选 …"');
  ok(!$('pickedPos').classList.contains('hidden')
     && $('pickedPos').classList.contains('set'),
     '选中态样式已加上');

  console.log('\n【19b】拖动地图不该误判成选点');
  window.eval('S.picked = null;');
  pd('pointerdown', 400, 300);
  await tick(10);
  pd('pointermove', 500, 380);
  await tick(10);
  pd('pointermove', 560, 420);
  await tick(10);
  pd('pointerup', 560, 420);
  await tick(10);
  ok(window.eval('S.picked') === null,
     '拖动是平移视图，不能顺手选一个点（否则每次拖地图都改坐标）');

  console.log('\n【19c】长按该设导航目标、且不算选点');
  calls.length = 0;
  window.eval('S.picked = null;');
  pd('pointerdown', 300, 250);
  await tick(750);                       // 超过 650ms 的长按阈值
  pd('pointerup', 300, 250);
  await tick(10);
  const goalCall = calls.find((c) => c.path === '/api/command'
                              && c.body && c.body.command === 'goto');
  ok(!!goalCall, '长按 650ms 设了导航目标');
  ok(window.eval('S.picked') === null, '长按不当作"选点位"');

  console.log('\n【20】小车图标：正方形 + 车头车尾标注（不再是三角形）');
  const drawSrc = js;
  ok(/ctx\.rect\(-L \/ 2, -W \/ 2, L, W\)/.test(drawSrc),
     '车体用 rect 画正方形（不再是 moveTo/lineTo 三角形）');
  ok(!/ctx\.moveTo\(0, -W \/ 2\);\s*\n\s*ctx\.lineTo\(L \/ 2, W \/ 2\)/.test(drawSrc),
     '旧的三角形三点路径已删除');
  ok(/strokeText\('车头'/.test(drawSrc) && /fillText\('车头'/.test(drawSrc),
     '车头有文字标注（带描边，压在白地图上也看得清）');
  ok(/strokeText\('车尾'/.test(drawSrc) && /fillText\('车尾'/.test(drawSrc),
     '车尾有文字标注');
  ok(/fillRect\(L \/ 2 - band, -W \/ 2, band, W\)/.test(drawSrc),
     '车头一侧涂了深色条（色弱/缩小时也能分前后）');
  ok(!/ctx\.rotate\(-pose\.yaw\)[\s\S]{0,400}fillText\('车头'/.test(
       drawSrc.split('ctx.restore();')[1] || ''),
     '车头车尾文字不跟着车体旋转（斜着走时也能读）');

  console.log('\n【21】路线：定时刷新 + 失败不清空本地列表');
  ok(/setInterval\(refreshRoutes/.test(js),
     '路线必须定时刷新（否则栈起来后永远看不到已有路线）');
  ok(/S\.routeCounts/.test(js), '记住每条路线的途经点数（选路线时显示）');
  const refreshFn = js.slice(js.indexOf('async function refreshRoutes'),
                             js.indexOf('function renderRoutes'));
  ok(/r\.ok === false/.test(refreshFn) && /return/.test(refreshFn),
     '后端 ok=false 时直接返回，不清空本地列表');
  ok(/Array\.isArray\(r\.names\)/.test(refreshFn),
     '只有真拿到 names 数组才覆盖本地列表');

  console.log('\n【22】存点位必须带 yaw（用户报："地图选点保存没有 yaw"）');
  // ★ 旧代码 saveWaypoint 里写死了 `yaw: 0.0`，且界面上完全不显示朝向，
  //   所以所有点位都"朝东站"，导航到位后车身朝哪全靠运气。
  //   这里真跑一遍：点地图 → 存点位 → 拖方向盘 → 检查提交的 yaw。
  calls.length = 0;
  window.eval("S.waypoints = []; S.picked = {x: 2, y: 3, yaw: 0};");
  const sw = window.saveWaypoint('cargo');
  await tick(40);
  ok(modalOpen(), '存点位弹框打开');
  const poseInputs = [...$('modalBody').querySelectorAll('input')];
  ok(poseInputs.length >= 3,
     `弹框里有名字 + x + y + yaw 输入（实际 ${poseInputs.length} 个 input）`);
  ok(!!$('modalBody').querySelector('.yaw-dial'),
     '有朝向方向盘（不是只有坐标）');
  // 点方向盘左侧 = 指西 = 180°
  const dial = $('modalBody').querySelector('.yaw-dial');
  const db = dial.getBoundingClientRect();
  dial.dispatchEvent(new window.MouseEvent('pointerdown', {
    clientX: db.left + 6, clientY: db.top + db.height / 2,
    bubbles: true, cancelable: true, view: window,
  }));
  await tick(20);
  const yawInp = $('modalBody').querySelector('input[aria-label="朝向 yaw 角度"]');
  // ★ 角度框里显示的是"度"，但 180° 归一化后是 ±180（弧度 π 用 normYaw
  //   归一化后可能落在负半轴），所以按绝对值比较。
  const shownDeg = Math.abs(parseFloat(yawInp.value));
  ok(shownDeg > 170 && shownDeg <= 180,
     `点方向盘左侧 → 角度显示接近 ±180°（实际 ${yawInp.value}°）`);
  ok(window.eval('S.preview') !== null,
     '调朝向时地图上有实时预览标记');
  clickModalOk();
  await sw;
  await tick(40);
  const swCall = calls.find((c) => c.path === '/api/store' && c.body
                                   && c.body.action === 'save');
  ok(!!swCall, '调用了 /api/store save');
  ok(swCall && Math.abs(swCall.body.yaw) > 2.6,
     `提交的 yaw 是调过的朝向，不再是写死的 0（实际 ${swCall && swCall.body.yaw}）`);
  ok(swCall && swCall.body.kind === 'cargo', 'kind 仍然正确');
  ok(window.eval('S.preview') === null, '保存后预览标记已清除');

  console.log('\n【23】手动输入方式添加点位（用户要求的新入口）');
  ok(typeof window.addWaypointManual === 'function',
     'addWaypointManual 可用');
  ok(/$('btnWpManual')/.test(js) || /btnWpManual/.test(js),
     '页面有「手动输入点位」按钮绑定');
  calls.length = 0;
  // 不点地图也能量：先把 picked 清空，默认值应回落到小车位姿/原点
  window.eval('S.picked = null; S.snap = {pose: {x: 1.5, y: -0.5, yaw: 0.5}};');
  const am = window.addWaypointManual();
  await tick(40);
  ok(modalOpen(), '手输弹框打开');
  const amInputs = [...$('modalBody').querySelectorAll('input')];
  const nameI = amInputs.find((i) => i.getAttribute('aria-label') === '点位名');
  const xI = amInputs.find((i) => i.getAttribute('aria-label') === 'x（米）');
  const yI = amInputs.find((i) => i.getAttribute('aria-label') === 'y（米）');
  ok(!!nameI && !!xI && !!yI, '有名字 / x / y 三个输入框');
  ok(Math.abs(parseFloat(xI.value) - 1.5) < 1e-6
     && Math.abs(parseFloat(yI.value) + 0.5) < 1e-6,
     `默认值回落到小车当前位姿（实际 x=${xI.value} y=${yI.value}）`);
  // 手打新坐标
  xI.value = '7.25'; xI.dispatchEvent(new window.Event('input'));
  yI.value = '-3.5'; yI.dispatchEvent(new window.Event('input'));
  await tick(10);
  clickModalOk();
  await am;
  await tick(40);
  const amCall = calls.find((c) => c.path === '/api/store' && c.body
                                   && c.body.action === 'save');
  ok(!!amCall, '手输后调用了 /api/store save');
  ok(amCall && amCall.body.x === 7.25 && amCall.body.y === -3.5,
     `提交的是手输坐标（实际 x=${amCall && amCall.body.x} y=${amCall && amCall.body.y}）`);

  console.log('\n【24】重新定位："用不了"的三个根因都要堵住');
  // ★ 旧实现：`const pose = S.snap.pose; if (!pose) return;` 然后
  //   api({x: pose.x, ...}) —— 把当前位姿原样发回去，等于没定位；
  //   而且没位姿时按钮直接 return，正是"定位飘了想重定位"时点不动。
  const relocFn = js.slice(js.indexOf('async function relocalize'),
                           js.indexOf('// ══════════════════════════════════════════════════════════════\n//  点位列表'));
  // ★ 用正向断言"发给后端的必须是编辑后的值"，比"不能出现 pose.x"稳：
  //   函数的注释里本来就会引用旧代码做对比，负向断言会被自己的注释绊倒。
  ok(/api\('\/api\/relocalize', \{x: v\.x, y: v\.y, yaw: v\.yaw\}\)/.test(relocFn),
     '发给 AMCL 的是弹框里编辑后的值（旧 bug：把当前位姿原样发回，等于没定位）');
  ok(/S\.picked/.test(relocFn), '优先用地图上选中的点当默认值');
  ok(/uiPose/.test(relocFn), '走位姿编辑弹框，用户可以真的改坐标');
  ok(/if \(S\.mode === 'mapping'\)/.test(relocFn),
     '建图模式下明确告知不需要重定位（那里没有 AMCL）');
  ok(!/if \(!pose\) \{ pushLog\('warn', '还没有位姿，无法重定位'\); return; \}/.test(js),
     '没有位姿时也允许点重定位（那正是最需要它的时候）');
  ok(/r\.ok === false/.test(relocFn) && /pushLog\('error'/.test(relocFn),
     '后端说没生效（无订阅者）时报错，不假装成功');

  calls.length = 0;
  window.eval("S.mode='nav'; S.picked=null; S.snap={pose:{x:0,y:0,yaw:0}};");
  const rl = window.relocalize();
  await tick(40);
  ok(modalOpen(), '重定位弹框打开');
  const rlX = $('modalBody').querySelector('input[aria-label="x（米）"]');
  rlX.value = '4.2'; rlX.dispatchEvent(new window.Event('input'));
  await tick(10);
  clickModalOk();
  await rl;
  await tick(40);
  const rlCall = calls.find((c) => c.path === '/api/relocalize');
  ok(!!rlCall, '调用了 /api/relocalize');
  ok(rlCall && rlCall.body.x === 4.2,
     `发给 AMCL 的是用户改过的坐标（实际 ${rlCall && rlCall.body.x}）`);

  // 建图模式：不该发请求，只提示
  calls.length = 0;
  window.eval("S.mode='mapping';");
  await window.relocalize();
  ok(!calls.some((c) => c.path === '/api/relocalize'),
     '建图模式下不白发 /api/relocalize');
  window.eval("S.mode='nav';");

  console.log('\n【25】切模式/换栈不能留重影、不能留旧小车位置');
  // ★ 用户报的两条：
  //   "从导航模式与建图模式相互切换时有遗留，导致网页地图显示有重影"
  //   "切换之后小车的位置显示依旧保留切换前的最后显示"
  //   根因都在"网关缓存只被新数据覆盖、从不作废"。
  ok(/map_epoch/.test(js), '前端认识 map_epoch（换栈事件）');
  ok(/'type': 'map_epoch'/.test(py),
     '后端换栈时下发 map_epoch 事件（与 map_changed 语义分开）');
  ok(/def reset_runtime_cache/.test(py), '后端有 reset_runtime_cache');
  ok(/function dropMapCache/.test(js), '前端有 dropMapCache');
  ok(/if \(changed\) \{\s*self\.reset_runtime_cache|reset_runtime_cache\(f'切到/.test(py)
     || /self\.reset_runtime_cache\(f'切到/.test(py),
     '模式真的变化时后端会清缓存');
  // 后端换了栈必须把这几样置空，一样漏了就会留残影
  const resetFn = py.slice(py.indexOf('def reset_runtime_cache'));
  for (const key of ['_gw_map_png', '_gw_map_meta', '_gw_pose', '_gw_scan', '_gw_plan']) {
    ok(resetFn.indexOf('self.' + key + ' = None') >= 0,
       `换栈清理包含 ${key}（漏了就会留残留）`);
  }
  ok(/_gw_map_epoch \+= 1/.test(resetFn), '换栈时地图代次 +1（前端靠它复位）');

  // 位姿保鲜期：TF 查不到不能无限沿用最后一帧
  ok(/pose_hold_sec/.test(py), '有位姿保鲜期参数');
  ok(/_gw_last_pose_ok/.test(py), '记录上次成功拿到位姿的时刻');
  const poseFn = py.slice(py.indexOf('def refresh_pose'),
                          py.indexOf('def query_pose_by_service'));
  ok(/_gw_pose = None/.test(poseFn),
     '位姿超时后对外报 None（否则小车永远停在切换前的位置）');

  // 前端行为：map_epoch 事件必须把旧地和旧位姿一起清掉
  window.eval("S.map.epoch = 0; S.map.img = {}; S.map.meta = {origin_x:0,origin_y:0,"
            + "width:10,height:10,resolution:0.05}; S.picked={x:1,y:1,yaw:0};"
            + "S.snap = {pose:{x:5,y:5,yaw:1}, scan:[[1,1]]}; loadMap = function(){};");
  window.eval('onMapEpoch(1)');
  await tick(10);
  ok(window.eval('S.map.img') === null, 'eopch 变化后旧地图图片被清掉（防重影）');
  ok(window.eval('S.map.meta') === null, '旧地图几何也清掉（否则按旧 origin 画）');
  ok(window.eval('S.picked') === null, '旧图上的选中点作废');
  ok(window.eval('S.snap') === null, '旧位姿/激光/路径作废');
  ok(window.eval('S.autoCenterDone') === false, '重新自动适配视图');

  // 位姿丢失时界面必须说"定位未就绪"，不能保留旧读数
  window.eval("S.snap={}; applyState({pose:null, task:{state:'IDLE'}, lease:{}, mode:'nav'});");
  await tick(10);
  ok(($('coord').textContent || '').indexOf('定位未就绪') >= 0,
     `拿不到位姿时明确提示（实际 "${$('coord').textContent}"）`);
  ok($('coord').classList.contains('stale'), '读数有 stale 样式');

  // 地图请求乱序：后发的请求赢，旧响应必须被丢掉
  ok(/seq !== S\.map\.seq/.test(js),
     '地图加载按请求序号丢弃过期响应（否则旧图会盖新图，也是重影来源）');
  ok(/cache:'no-store'|t=\$\{Date\.now\(\)\}/.test(js)
     || /t=\$\{Date\.now\(\)\}/.test(js),
     '地图请求带时间戳防手机浏览器缓存');

  console.log('\n【26】点位列表：list 一次拿全，不再 N+1 逐个 get');
  ok(/Array\.isArray\(r\.items\)/.test(js),
     '优先用 list 返回的 items（旧代码 10 个点位要发 11 次 ROS 服务调用）');

  // ══════════════════════════════════════════════════════════
  //  【27】实时仪表：固定 7 槽位，值来自状态量
  //
  //  用户原话（第二轮，本组测试重写的原因）：
  //    "数据面板显示的日志是固定死的，日志长的话看不到后面的信息。……
  //     数据面板，就显示一些重点状态标志。"
  //  ★ 上一版这里断言的是"格子按 key 增删、覆盖显示、带 ×N 计数"，
  //    全是"日志镜像"的行为。现在面板是**固定 8 行仪表**：
  //    行数恒定、位置恒定、值由后端 build_gauges 从状态量算出。
  //    所以测试也换成守新契约，否则测试通过了功能却不对。
  // ══════════════════════════════════════════════════════════
  console.log('\n【27】实时仪表：固定 7 槽位，值来自状态量');

  // ── 后端契约 ──
  ok(/_gw_logf\.feed\(/.test(py), '后端 log() 会先把日志交给降噪器判定');
  ok(/'board': build_gauges\(gstate\)/.test(py),
     'snapshot 的 board 来自 build_gauges（值取自状态量，不是日志文本）');
  ok(!/StatusBoard/.test(py),
     '★ 已不再用旧的 StatusBoard（日志镜像）—— 那正是"固定死"的根源');
  ok(/create_subscription\(Twist, gp\('vel_topic'\)\.value/.test(py),
     '网关订阅了底盘实际速度（仪表"速度"格的数据来源）');
  const resetFn2 = py.slice(py.indexOf('def reset_runtime_cache'));
  ok(/_gw_logf\.reset\(\)/.test(resetFn2),
     '换栈时清空限流状态（否则新栈第一条避障被旧窗口吞掉）');
  ok(/_gw_speed = None/.test(resetFn2),
     '换栈时清空速度（否则仪表挂着旧栈的速度，车根本没动）');

  // ── 前端结构：固定 8 行 ──
  window.eval('clearBoard()');
  await tick(10);
  ok(document.querySelectorAll('#board .brow').length === 0, 'clearBoard 清空所有行');

  // 后端没给数据时也要画出 8 行骨架（不能空着让人以为功能没做）
  window.eval('renderBoard([])');
  await tick(10);
  ok(document.querySelectorAll('#board .brow').length === 8,
     `空数据时按骨架画 8 行，实际 ${document.querySelectorAll('#board .brow').length} 行`);
  // ★ 顺序必须和后端 GAUGES 一致：'动作' 紧跟 '任务'（载货状态是安全信息），
  //   '连接' 挪到最后（只在断线时才需要看）。任一侧改了顺序，这里立刻红。
  const skeletonKeys = ['task', 'act', 'speed', 'avoid', 'pose', 'map', 'mode', 'link'];
  ok(skeletonKeys.every((k) => !!document.getElementById('bs-' + k)),
     '8 个槽位 key 齐全且顺序固定（task/act/speed/avoid/pose/map/mode/link）');
  const firstRow = document.getElementById('bs-task');
  ok(firstRow.querySelector('.bv').textContent === '——',
     '无数据显示占位"——"（不能显示空白，那是两回事）');

  // ── 渲染行为：行数恒定、节点复用、只改文字 ──
  const mkBoard = (vals) => skeletonKeys.map((k, i) => ({
    key: k, label: ['任务', '动作', '速度', '避障', '定位', '地图', '模式', '连接'][i],
    text: vals[i], tone: 'ok',
  }));
  window.eval('renderBoard(' + JSON.stringify(mkBoard(
    ['空闲', '无动作', '0.30 · -0.28', '1.20m 畅通', '正常', '594×762', '导航 · 自动', '已连接']
  )) + ')');
  await tick(10);
  ok(document.querySelectorAll('#board .brow').length === 8, '喂 8 条仍是 8 行');
  ok(document.getElementById('bs-speed').querySelector('.bv').textContent
     === '0.30 · -0.28', '速度值正确渲染');
  ok(document.getElementById('bs-act').querySelector('.bv').textContent === '无动作',
     '★ 动作格渲染"无动作"');

  // 40 帧持续刷新：节点必须复用，行数不能变
  const nodeBefore = document.getElementById('bs-avoid');
  for (let i = 0; i < 40; i++) {
    window.eval('renderBoard(' + JSON.stringify(mkBoard(
      ['搬运中 2/5', '载货中', `0.${20 + i} · -0.28`, `${(0.5 + i * 0.01).toFixed(2)}m 减速`,
       '正常', '594×762', '导航 · 自动', '已连接']
    )) + ')');
  }
  await tick(10);
  ok(document.querySelectorAll('#board .brow').length === 8,
     '★ 40 帧刷新后仍是 8 行（固定槽位不会越堆越多）');
  ok(document.getElementById('bs-avoid') === nodeBefore,
     '★ 节点被复用而不是重建（重建会让面板每帧闪一下）');
  ok(document.getElementById('bs-act').querySelector('.bv').textContent === '载货中',
     '★ 动作格能刷新成"载货中"（车上有没有货必须实时可见）');
  ok(document.getElementById('bs-avoid').querySelector('.bv').textContent.indexOf('0.89') >= 0,
     `文字被替换成最新值（实际 "${document.getElementById('bs-avoid').querySelector('.bv').textContent}"）`);
  ok(document.getElementById('bs-task').querySelector('.bv').textContent === '搬运中 2/5',
     '任务格显示进度');

  // ── 色调映射 ──
  window.eval('renderBoard(' + JSON.stringify([
    {key: 'exec', label: '任务', text: '急停中', tone: 'error'},
  ]) + ')');
  await tick(10);
  ok(document.getElementById('bs-exec').className.indexOf('tone-error') >= 0,
     'error 色调落到 class 上');
  ok(document.getElementById('boardBadge').className.indexOf('warn') >= 0,
     '有异常时徽标变黄');
  ok(document.getElementById('boardBadge').textContent.indexOf('注意') >= 0,
     `徽标文字提示异常（实际 "${document.getElementById('boardBadge').textContent}"）`);

  // ── 用户核心诉求：值不能长到看不全 ──
  //  只检查**生效的 CSS 声明**，不检查注释：注释里提到 line-clamp 是在
  //  解释"为什么删掉它"，不该被判成违规（第一版测试就误报在这）。
  const cssNoComments = css.replace(/\/\*[\s\S]*?\*\//g, '');
  ok(!/line-clamp/.test(cssNoComments),
     '★ CSS 不再用 line-clamp 截断（那正是"日志长看不到后面"的元凶）');
  ok(!/\.board\{[^}]*overflow-y/.test(cssNoComments),
     '★ 面板不再需要滚动（8 行固定，滚动就等于看不全）');

  // ── applyState 必须把 board 喂进渲染 ──
  ok(/renderBoard\(st\.board\)/.test(js),
     'applyState 会把后端下发的仪表数据渲染出来');

  console.log('\n' + '─'.repeat(56));
  console.log(`  通过 ${pass} 项，失败 ${fail} 项`);

  if (fail) {
    console.log('  失败项：');
    failures.forEach((f) => console.log('    · ' + f));
  }
  console.log('─'.repeat(56));
  process.exit(fail ? 1 : 0);
})().catch((e) => {
  console.log('\n✗ 测试过程抛异常: ' + e.stack);
  process.exit(1);
});

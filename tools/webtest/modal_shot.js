// 用 jsdom 渲染模态框，导出一张可直接用浏览器打开的 HTML 预览
const fs = require('fs');
const path = require('path');
const { JSDOM } = require('jsdom');

const ROOT = '/home/yuki/agv/newros_car7/src/agv_web/agv_web/static';
const html = fs.readFileSync(path.join(ROOT, 'index.html'), 'utf8');
const js = fs.readFileSync(path.join(ROOT, 'app.js'), 'utf8');
const css = fs.readFileSync(path.join(ROOT, 'app.css'), 'utf8');

const dom = new JSDOM(html, {
  url: 'http://127.0.0.1:8080/',
  runScripts: 'dangerously',
  pretendToBeVisual: true,
  beforeParse(w) {
    w.HTMLCanvasElement.prototype.getContext = () => new Proxy({}, {
      get: () => () => ({ width: 10 }),
      set: () => true,
    });
    Object.defineProperty(w.HTMLElement.prototype, 'clientWidth', { get() { return 390; } });
    Object.defineProperty(w.HTMLElement.prototype, 'clientHeight', { get() { return 500; } });
    w.HTMLElement.prototype.getBoundingClientRect = () => ({
      left: 0, top: 0, width: 390, height: 500,
    });
    w.fetch = async () => ({
      ok: true, status: 200,
      json: async () => ({ ok: true, logs: [], items: [], names: [], tasks: [], maps: [] }),
    });
  },
});

const { window } = dom;
const { document } = window;

const el = document.createElement('script');
el.textContent = js;
document.body.appendChild(el);
document.dispatchEvent(new window.Event('DOMContentLoaded'));

(async () => {
  // 打开"选途经点"多选模态框 —— 最能体现触屏改进的界面
  window.uiMulti(
    '选择途经点',
    '路线「厂房A环线」：按顺序点选经过的点位，第一个是起点、最后一个是终点',
    [
      { value: '货架A', label: '货架A' },
      { value: '充电桩', label: '充电桩' },
      { value: '库位1', label: '库位1' },
      { value: '出库口', label: '出库口' },
    ],
  );
  await new Promise((r) => setTimeout(r, 60));

  // 模拟点了第 1 个和第 3 个
  document.querySelectorAll('#modalBody .opt')[0].click();
  document.querySelectorAll('#modalBody .opt')[2].click();
  await new Promise((r) => setTimeout(r, 60));

  const bodyHtml = document.body.innerHTML;
  const out = '<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">'
    + '<meta name="viewport" content="width=device-width,initial-scale=1">'
    + '<style>' + css + '</style></head><body>' + bodyHtml + '</body></html>';
  fs.writeFileSync('/home/yuki/agv/newros_car7/tools/webtest/modal_preview.html', out);
  console.log('已生成 modal_preview.html');

  console.log('\n模态框实际结构：');
  const title = document.getElementById('modalTitle').textContent;
  const desc = document.getElementById('modalDesc').textContent;
  console.log('  标题:', title);
  console.log('  说明:', desc.slice(0, 42) + '…');

  const items = Array.from(document.querySelectorAll('#modalBody .opt'));
  items.forEach((o, i) => {
    const isOn = o.classList.contains('on');
    const ord = o.querySelector('.ord');
    const state = isOn ? ('选中 #' + (ord ? ord.textContent : '?')) : '未选';
    console.log('  [' + i + '] ' + state + '  ' + o.querySelector('.nm').textContent.trim());
  });

  const tip = document.querySelector('#modalBody .hint');
  console.log('  底部提示:', tip ? tip.textContent : '(无)');

  // 断言（失败则退出码非 0）
  let bad = 0;
  if (title !== '选择途经点') { console.log('  ✗ 标题不对'); bad++; }
  if (items.length !== 4) { console.log('  ✗ 列表项数不对'); bad++; }
  const onCount = items.filter((o) => o.classList.contains('on')).length;
  if (onCount !== 2) { console.log('  ✗ 选中数应为 2，实际 ' + onCount); bad++; }
  if (!tip || !tip.textContent.includes('货架A → 库位1')) {
    console.log('  ✗ 底部顺序提示不对：' + (tip && tip.textContent));
    bad++;
  }
  console.log(bad ? '\n  ✗ 有 ' + bad + ' 项断言失败' : '\n  ✓ 全部断言通过');
  process.exit(bad ? 1 : 0);
})();

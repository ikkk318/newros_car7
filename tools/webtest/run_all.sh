#!/usr/bin/env bash
# ============================================================================
#  AGV 网页控制台 —— 一键跑完所有验证
#
#  用法（两条命令，复制粘贴即可）：
#      cd /home/yuki/agv/newros_car7
#      bash tools/webtest/run_all.sh
#
#  只跑某一组：
#      bash tools/webtest/run_all.sh unit      # 只跑 69 项 Python 单测（最快，2 秒）
#      bash tools/webtest/run_all.sh ui        # 只跑 63 项前端交互测试
#      bash tools/webtest/run_all.sh e2e       # 只跑端到端（起网关+注入器，约 60 秒）
#      bash tools/webtest/run_all.sh mapping   # 只跑建图模式遥控话题分流专项
#      bash tools/webtest/run_all.sh render    # 只出地图渲染预览图
#
#  ★ 不能加 set -u：ROS 的 setup.bash 会读未绑定变量（AMENT_TRACE_SETUP_FILES），
#    一开就报 "未绑定的变量" 直接退出。这个坑踩过，注释留在这里。
# ============================================================================

WS=/home/yuki/agv/newros_car7
cd "$WS" || { echo "找不到工作目录 $WS"; exit 1; }

# ★ python3 必须是 /usr/bin 的 3.10（PATH 里默认那个是 3.13，没装 numpy，
#   会让 ROS 的 spawn_entity.py 直接崩）。所以先把 /usr/bin 提到最前面。
export PATH=/usr/bin:/bin:/usr/local/bin:$PATH
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || echo "（install/ 还没编译，先跑 colcon build）"

export no_proxy='*'          # 系统代理会拦 127.0.0.1，导致 curl 返回 502
export NO_PROXY='*'

GROUP="${1:-all}"
PASS=0
FAIL=0

hr() { echo; echo "════════════════════════════════════════════════════════════════"; }

ok()   { PASS=$((PASS+1)); echo "  ✅ $1"; }
bad()  { FAIL=$((FAIL+1)); echo "  ❌ $1"; }

# ────────────────────────────────────────────────────────────────────────────
# 1) 编译
# ────────────────────────────────────────────────────────────────────────────
do_build() {
  hr; echo "【1/6】编译 agv_web"; hr
  if colcon build --packages-select agv_web --symlink-install 2>&1 | tail -3 | grep -q "Summary"; then
    ok "编译通过"
  else
    bad "编译失败（看上面输出）"
  fi
}

# ────────────────────────────────────────────────────────────────────────────
# 2) Python 单测（69 项）
# ────────────────────────────────────────────────────────────────────────────
do_unit() {
  hr; echo "【2/6】Python 单测"; hr
  local out
  # ★ 把 agv_common 也带上：地图库（map_manager）的改动必须一起回归，
  #   否则网关那边"引用了一个不存在的字段"这种错会被漏过。
  # ★ 四个包都要跑：agv_waypoint 的"文件外部改动自动重载"、
  #   agv_task 的"中止不落 ERROR"都是真机 bug 的回归锁，
  #   只跑 web/common 会漏掉一半。
  out=$(/usr/bin/python3 -m pytest src/agv_web/test/ src/agv_common/test/ \
        src/agv_waypoint/test/ src/agv_task/test/ -q 2>&1 | tail -3)
  echo "$out"
  # ★ 不要写死"69 passed"：测试会随修 bug 增加（现在是 79），
  #   写死就会在代码完全正确时报失败，比没有检查更坑。
  #   判据改成"没有 failed/error 且有 passed"。
  if echo "$out" | grep -qE "failed|error"; then
    bad "单测有失败"
  elif echo "$out" | grep -q "passed"; then
    ok "$(echo "$out" | grep -oE '[0-9]+ passed')"
  else
    bad "单测没跑起来（看上面输出）"
  fi
}

# ────────────────────────────────────────────────────────────────────────────
# 3) 前端交互（63 项，jsdom 真跑 app.js）
# ────────────────────────────────────────────────────────────────────────────
do_ui() {
  hr; echo "【3/6】前端交互（jsdom 真跑）"; hr
  # ★ NODE_PATH 必须指到装了 jsdom 的目录，否则 MODULE_NOT_FOUND
  local NODE=/home/yuki/.workbuddy/binaries/node/versions/22.22.2/bin/node
  local NM=/home/yuki/.workbuddy/binaries/node/workspace/node_modules
  if [ ! -x "$NODE" ]; then
    echo "  找不到 node（$NODE），跳过"; return
  fi
  local out
  out=$(NODE_PATH=$NM "$NODE" tools/webtest/ui_test.js 2>&1)
  echo "$out" | tail -6
  # 同理不写死项数，只判"失败 0 项"
  echo "$out" | grep -q "失败 0 项" && ok "前端全绿" || bad "前端测试有失败"
}

# ────────────────────────────────────────────────────────────────────────────
# 4) JS 语法 + 原生弹窗残留
# ────────────────────────────────────────────────────────────────────────────
do_syntax() {
  hr; echo "【4/6】静态检查（语法 / 原生弹窗残留）"; hr
  local NODE=/home/yuki/.workbuddy/binaries/node/versions/22.22.2/bin/node
  if "$NODE" --check src/agv_web/agv_web/static/app.js 2>/dev/null; then
    ok "app.js 语法通过"
  else
    bad "app.js 语法错误"
  fi
  # ★ 注意排除注释行：源码里第 61/63/64/72 行是讲"为什么换掉 prompt()"的说明文字，
  #   不算真实调用。不加 grep -v 排除注释会误报 4 处。
  local n
  n=$(grep -n "prompt(\|confirm(" src/agv_web/agv_web/static/app.js \
      | grep -vc "^\s*[0-9]*://")
  if [ "$n" -eq 0 ]; then
    ok "原生弹窗真实调用 0 处（已全换触控模态框）"
  else
    bad "还有 $n 处原生弹窗没换"
  fi
}

case "$GROUP" in
  all)     do_build; do_unit; do_ui; do_syntax ;;
  unit)    do_build; do_unit ;;
  ui)      do_ui; do_syntax ;;
  build)   do_build ;;
  syntax)  do_syntax ;;
  e2e)     echo "→ 转交 final.sh（端到端，约 60 秒）"; bash tools/webtest/final.sh ;;
  mapping) echo "→ 转交 mapping_teleop.sh（建图模式遥控话题专项）"; bash tools/webtest/mapping_teleop.sh ;;
  reset)   echo "→ 转交 stack_reset.sh（切模式后重影/旧位姿专项，约 90 秒）"
           bash tools/webtest/stack_reset.sh ;;
  sim)     echo "→ 转交 sim_pose.sh（仿真时基下小车是否显示，约 60 秒）"
           bash tools/webtest/sim_pose.sh ;;
  board)   echo "→ 跑 board_e2e.py（仪表 8 槽位 + 日志降噪 + 跨节点转发，约 20 秒）"
           # ★ 这个不需要起完整 ROS 栈：它在进程内建网关节点 + 真 aiohttp，
           #   灌 200 条真实格式的避障日志，断言：
           #     · 仪表是固定 7 槽位、值够短（不再"看不全"）
           #     · 避障被限流到 5 秒一条（不是全丢也不是全留）
           #     · 顶升/下降/完成这类事件一条不少
           source /opt/ros/humble/setup.bash 2>/dev/null
           source install/setup.bash 2>/dev/null
           no_proxy='*' NO_PROXY='*' /usr/bin/python3 tools/webtest/board_e2e.py ;;
  gauge)   echo "→ 出图 tools/webtest/gauge_preview.png（仪表观感预览）"
           /usr/bin/python3 tools/webtest/gauge_preview.py ;;
  task)    echo "→ 转交 task_multi.sh（多任务连续执行 + 中止恢复，约 2 分钟）"
           bash tools/webtest/task_multi.sh "${2:-3}" ;;
  task1)   echo "→ 转交 task_cycle.sh（单任务闭环：开始→中止→再启动）"
           bash tools/webtest/task_cycle.sh ;;
  render)  echo "→ 出图 tools/webtest/render_preview.png"
           /usr/bin/python3 tools/webtest/render_preview.py ;;
  *)
    echo "未知参数：$GROUP"
    echo "可用：all | unit | ui | syntax | build | e2e | mapping | reset | task | task1 | render"
    exit 2 ;;
esac

if [ "$GROUP" = "all" ] || [ "$GROUP" = "unit" ] || [ "$GROUP" = "ui" ] || [ "$GROUP" = "syntax" ]; then
  hr
  echo "结果：通过 $PASS 组，失败 $FAIL 组"
  [ "$FAIL" -eq 0 ] && echo "✅ 全部通过" || echo "❌ 有失败，看上面输出"
  hr
fi

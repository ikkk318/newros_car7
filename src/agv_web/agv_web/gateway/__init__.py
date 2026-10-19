"""`agv_web` 的网关实现包：按"状态 / 动作 / 传输"拆开，`web_gateway.py` 只留入口。

依赖方向（单向，别反向 import，否则又会绕回一个大文件）：

    util.py   ←  所有模块（叶子：不 import 本包里任何东西）
    stack.py  ←  util
    state.py  ←  util
    commands.py ← util, stack_manager
    node.py   ←  state, commands, stack, util          （组装 WebGateway）
    web_app.py ← handlers_ops, handlers_data, util     （装配 aiohttp 应用）
    web_gateway.py ← node, web_app, stack, util        （入口）

★ 本 `__init__.py` **故意不 import 任何子模块**：
  aiohttp 是可选的（缺了要给中文提示而不是 traceback），
  如果在包初始化时就 import `web_app`，那个"可选"就没了。
"""

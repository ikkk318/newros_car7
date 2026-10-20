# 版本管理与发布

> 回答一个问题：**为什么传完 GitHub 看起来"上一次的没了"**，以及怎么让它
> 像正常软件那样一版一版往上叠。

---

## 1 GitHub 只是"默认给你看最新一张照片"

git 的存储模型是**追加的**：每次 `commit` 都在历史末尾接一个节点，指向它的
父节点，**永远不会删除或改写前面的节点**。`push` 只是把这些节点复制到远端。
所以从机制上讲，"覆盖"在正常推送下不可能发生。

那为什么看起来像覆盖了？因为 GitHub 的默认视图只展示**最新快照**：

| 你看到的 | 它实际的含义 |
|---|---|
| 仓库首页的文件列表 | **最新提交**里的文件。你删掉的文件当然不在这里 |
| 点进某个文件 | 该文件在最新提交里的内容 |
| 目录里少了一半文件 | 你在某个提交里删了它们，不是"被同一个文件覆盖" |

要看到"一版一版"，得点进 **Commits**（提交列表）或 **Tags**（版本标记）。
这两个入口 GitHub 都放在代码页右上角，很容易被忽略。

**可以这样验证**：在 GitHub 仓库地址后加 `/commits/main`，就能看到全部提交。
本项目现在是 8 条，从 `86706ea 建库` 一路到 `0a5a3b4 退出修复`。

---

## 2 本项目的实际历史（已核对远程）

```
0a5a3b4  fix(agv_web): 退出时等 spin 线程结束再销毁节点   ← v0.2.0
29ce4cc  chore: 清理最后的死码
b1eb342  refactor(web): app.js 拆成四个脚本
b5d34ea  refactor(agv_web): 网关拆成 gateway/ 8 模块
295ee0a  文档：整合为 README + docs 6 篇
4a7feb4  清理：删除全部测试与手工验收脚本
d8b5831  文档：GIT使用.md 改写                        ← v0.1.0（首次上传）
86706ea  建库：把 newros_car7 纳入版本控制
```

两次推送都是 **fast-forward**（快进），不是强推：

- 第一次 `d8b5831` 推上去时，远端从空变成 `d8b5831`
- 第二次 `0a5a3b4` 推上去时，远端从 `d8b5831` 快进到 `0a5a3b4`，
  前面那 7 个提交原封不动留在链上

验证命令（`YES` 就说明历史完整）：

```bash
git merge-base --is-ancestor 86706ea origin/main && echo YES
```

---

## 3 只有这几种操作会真的"抹掉"历史

| 操作 | 后果 | 什么时候用 |
|---|---|---|
| `git push --force` / `-f` | 远端分支指针被强行挪到另一条链，旧提交变"孤儿" | **几乎永远不要用** |
| `git reset --hard` + 强推 | 同上的典型组合 | 同上 |
| 重新 `git init` 后推同一个仓库 | 本地是全新历史，与远端无共同祖先 → 要么被拒，要么强推覆盖 | 不要这么做 |
| 删除重建 GitHub 仓库 | 真的没了 | 不要这么做 |

本项目从未用过 force push。以后也不要用——需要撤销时用 `git revert`（见 §6）。

---

## 4 版本号与标签：给"照片"贴名字

commit 号（`0a5a3b4`）是机器读的，标签（tag）是人读的。给阶段性成果打标签，
GitHub 上就会出现 **Releases / Tags** 页面，能一版一版对比、下载对应源码包。

### 语义化版本 `v主.次.修`

| 位 | 什么时候 +1 | 本项目例子 |
|---|---|---|
| 主 | 不兼容的接口变更（别人按旧文档写的东西跑不通了） | 目前没有，仍是 0.x |
| 次 | 加功能、改结构，接口仍兼容 | `v0.1.0` → `v0.2.0`（拆模块） |
| 修 | 只修 bug | `v0.2.1` |

### 已有标签

| 标签 | 指向 | 内容 |
|---|---|---|
| `v0.1.0` | `d8b5831` | 首次上传的完整状态：测试齐全、网关 3236 行单文件、15 份文档 |
| `v0.2.0` | `0a5a3b4` | 结构重构：文档 7 份、无测试、后端 8 模块、前端 4 脚本 |

### 打一个标签并发布

```bash
# 1. 附带说明的标签（推荐，-a 会把说明和作者时间一起存进去）
git tag -a v0.2.1 -m "修复：xxx 导致的 yyy"

# 2. 只推这一个标签
git push origin v0.2.1

# 3. 或一次推全部标签
git push origin --tags
```

> ⚠️ 标签**不会**跟着普通 `git push` 上传，必须显式推。这是最常见的
> "我明明打了标签，GitHub 上没有"的原因。

### 标签打错了怎么改

```bash
git tag -d v0.2.1                 # 删本地
git push origin :refs/tags/v0.2.1 # 删远端
git tag -a v0.2.1 <正确commit> -m "…"
git push origin v0.2.1
```

---

## 5 日常节奏（够用即可）

```bash
# 每完成一件事并验证过 → 一次提交
git add -A
git commit -m "fix(agv_task): 中止后回 IDLE，不再卡执行中"

# 推到 GitHub（第一次之后 git 会记住分支，不用再写 origin main）
git push

# 一个阶段收尾（比如"能跑通一次搬运"）→ 打标签
git tag -a v0.3.0 -m "…" && git push origin v0.3.0
```

**push 的频率不重要，commit 的粒度才重要。** 一次提交只做一件事：
改结构就别顺手修 bug，修 bug 就别顺手重排版。这样将来 `git revert` 才干净。

---

## 6 后悔药：三种场景对应的正确命令

```bash
# ① 只想看某个旧版本的内容（不动工作区）
git show v0.1.0:src/agv_web/agv_web/web_gateway.py | head -40

# ② 把某个已删文件拿回来
git checkout v0.1.0 -- src/agv_web/test/test_stack_cleanup.py

# ③ 整个工作区回到某个标签（当前改动会被丢弃，先确认没有未提交的东西）
git switch --detach v0.1.0

# ④ 已经推上去的提交要撤销 → 用 revert 生成一个"反向提交"，
#    历史保留、可追溯，不需要 force push
git revert 29ce4cc
git push
```

本项目删掉的 51 个测试文件、`tools/webtest/` 整目录都还在 `v0.1.0` 里，
需要哪份就按 ② 取回。

---

## 7 环境相关（本项目踩过的）

- **git 的代理配置与终端环境变量是两套**。本项目曾配了
  `http.proxy=http://127.0.0.1:7897`，代理换端口后 `git fetch` 就报
  `gnutls_handshake() failed`，但 `curl` 正常。修复：
  ```bash
  git config --global --unset http.proxy
  git config --global --unset https.proxy
  ```
  （已清除。现在 git 直连，能通就别再设。）
- **推送要认证**。`GIT_TERMINAL_PROMPT=0` 下报
  `could not read Username for 'https://github.com'`，说明当前没有缓存凭据，
  需要交互式输入一次，或改用 SSH。**在非交互环境里跑 push 会失败，属正常。**
- `http.version=HTTP/1.1` 与 `http.postBuffer=524288000` 已配在全局，
  大仓库推不上去时先别改它们。

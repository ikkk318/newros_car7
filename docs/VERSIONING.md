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
本项目现在是 9 条，从 `86706ea 建库` 一路到 `b85b1dc 补版本文档`。

> **两个症状要分开治，别混为一谈**：
> - **文件列表变了 / 少了文件** → 正常，GitHub 在展示最新快照，历史都在。
> - **Tags 页面空白** → 另一回事，是标签压根没推到远程（见 §4.1）。

---

## 2 本项目的实际历史（已核对远程）

```
b85b1dc  docs: 补 VERSIONING.md 与版本标签
0a5a3b4  fix(agv_web): 退出时等 spin 线程结束再销毁节点   ← v0.2.0
29ce4cc  chore: 清理最后的死码
b1eb342  refactor(web): app.js 拆成四个脚本
b5d34ea  refactor(agv_web): 网关拆成 gateway/ 8 模块
295ee0a  文档：整合为 README + docs 6 篇
4a7feb4  清理：删除全部测试与手工验收脚本
d8b5831  文档：GIT使用.md 改写                        ← v0.1.0（首次上传）
86706ea  建库：把 newros_car7 纳入版本控制
```

**三次推送都是 fast-forward（快进），一次 force push 都没有过**：

| 时间 | 远端 main | 说明 |
|---|---|---|
| 10-19 19:46 | 空 → `d8b5831` | 首次上传 |
| 10-19 22:54 | `d8b5831` → `0a5a3b4` | 7 个提交依次叠加，前面的原封不动 |
| 10-20 11:42 | `0a5a3b4` → `b85b1dc` | 同上 |

也就是说：**从建库到现在，任何一个历史文件都还能取回来**。
GitHub 文件列表变了只说明"最新快照里没有它"，不说明"它被覆盖了"。

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

**最稳的是一条命令（推荐）**：

```bash
bash tools/release.sh v0.2.1 "修复：中止后卡执行中"
```

脚本会先推未推送的提交、再打 annotated 标签、**显式推该标签**，最后回读远程
确认真的上去了。只看本地/远程对照：`bash tools/release.sh --list`。

手工做同样的事：

```bash
git tag -a v0.2.1 -m "修复：xxx 导致的 yyy"   # -a 才有说明和作者时间
git push origin refs/tags/v0.2.1              # ★ 显式推这个标签
git ls-remote --tags origin                   # 回读：远程真有才算成功
```

---

## 4.1 ★ 为什么 Tags 页面会是空的（本项目真实踩过）

2026-10-20：本地已经打好两个标签，`git push origin main` 也显示成功，
但 GitHub `/tags` 页面空白、`api.github.com/.../tags` 返回 `[]`。
**commit 一个没丢，只是标签没推上去。** 原因有三层，任何一层都会导致空页面：

| 坑 | 说明 |
|---|---|
| **`git push` 默认不传标签** | 只推当前分支。标签是独立引用，必须显式 `git push origin <tag>` |
| **`--follow-tags` 有前提** | 它**只推 annotated 标签**，且**只推指向当前提交或其后代的**。轻量标签（`git tag v1`，不带 `-a`）永远不被带走；标签若指向较早的提交，也可能被跳过 |
| **推送失败会被忽略** | 标签推送需要认证，一旦凭证没缓存/网络断，`git push --follow-tags` 可能在 commit 推成功后、标签阶段失败，而人只看到前面那行成功 |

**排查顺序（照做三行定位）**：

```bash
# ① 本地有没有
git tag -l

# ② 远程有没有（这才是 GitHub 页面显示的依据）
git ls-remote --tags origin

# ③ 本地有、②为空 → 就是没推上去，一条命令全补推
bash tools/release.sh --push-all
```

不想用脚本也行：

```bash
git push origin refs/tags/v0.1.0 refs/tags/v0.2.0   # 指名推两个
git push origin --tags                              # 推全部（含实验性的，慎用）
```

**关于 Releases**：Releases 是**建立在标签之上的发布说明**，没有标签就永远没有
Releases。标签推上去之后，Tags 页立刻有内容；想要带正文的 Release，
去 GitHub 的 `Releases → Draft a new release → 选已有 tag` 填写，或用
`gh release create v0.2.0 --notes-file docs/CHANGELOG.md`。

### 标签打错了怎么改

```bash
git tag -d v0.2.1                 # 删本地
git push origin :refs/tags/v0.2.1 # 删远端（冒号前缀 = 删除）
git tag -a v0.2.1 <正确commit> -m "…"
git push origin refs/tags/v0.2.1
```

---

## 5 日常节奏（够用即可）

```bash
# 每完成一件事并验证过 → 一次提交
git add -A
git commit -m "fix(agv_task): 中止后回 IDLE，不再卡执行中"

# 推到 GitHub（第一次 push -u 之后，git 记住上游，不用再写 origin main）
git push

# 一个阶段收尾（比如"能跑通一次搬运"）→ 用脚本打标签+推送+回读校验
bash tools/release.sh v0.3.0 "能跑通完整一次搬运"
```

**push 的频率不重要，commit 的粒度才重要。** 一次提交只做一件事：
改结构就别顺手修 bug，修 bug 就别顺手重排版。这样将来 `git revert` 才干净。

**标签不是每次提交都要打。** 只在"这一版可以拿给别人/给老师看/自己回滚到"
的时候打。打太多等于没有版本概念。

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

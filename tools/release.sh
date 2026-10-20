#!/usr/bin/env bash
# =============================================================================
# 打标签并推到 GitHub —— 一条命令搞定，避免"标签推不上去"的老坑
#
# 为什么需要这个脚本：
#   1. `git push` 默认**不传标签**。标签要显式推，否则 GitHub 的 Tags 页面
#      永远是空的（本项目已经踩过一次：commit 全上去了，tags API 返回 []）。
#   2. `--follow-tags` 只推** annotated **标签，且只推"当前提交或它祖先上的"。
#      轻量标签（`git tag v1`，不带 -a）永远不被 --follow-tags 带走。
#   3. 所以最稳的是：显式 `git push origin <tag>`，或用本脚本一次做完。
#
# 用法：
#   bash tools/release.sh v0.2.1 "修复：中止后卡执行中"    # 给当前 HEAD 打标签并推送
#   bash tools/release.sh v0.2.1                          # 说明省略则用最近一次提交标题
#   bash tools/release.sh --push-all                      # 本地标签全补推（Tags 页空时用）
#   bash tools/release.sh --list                          # 只看本地/远程标签对照
# =============================================================================
set -uo pipefail

cd "$(dirname "$0")/.." || exit 1

# ── 只看对照表 ────────────────────────────────────────────────────────────────
if [ "${1:-}" = "--list" ]; then
    echo "本地标签："
    git tag -l -n1 | sed 's/^/  /' || true
    echo
    echo "远程标签（GitHub 上真正存在的）："
    # 注意：不能写成 `ls-remote | grep -v ... || echo 取不到`。
    # 远程确实一个标签都没有时，grep 过滤后是空输入、退出码 1，会被误判成
    # "网络取不到" —— 这两种情况必须以 ls-remote 自己的退出码区分。
    if REMOTE_TAGS="$(git ls-remote --tags origin 2>/dev/null)"; then
        LIST="$(printf '%s\n' "$REMOTE_TAGS" | sed 's#.*refs/tags/##' | grep -v '\^{}$' || true)"
        if [ -z "$LIST" ]; then
            echo "  （空 —— GitHub 上一个标签都没有）"
        else
            printf '%s\n' "$LIST" | sed 's/^/  /'
        fi
    else
        echo "  （取不到，检查网络/代理）"
    fi
    echo
    echo "提示：本地有、远程没有 → 就是没推上去。用本脚本或"
    echo "      git push origin <标签名> 补推。"
    exit 0
fi

# ── 补推模式：把本地有、远程没有的标签全部推上去 ──────────────────────────────
# 这是"打完标签但 GitHub 上是空的"最直接的解法。
if [ "${1:-}" = "--push-all" ]; then
    LOCAL_TAGS="$(git tag -l)"
    if [ -z "$LOCAL_TAGS" ]; then
        echo "本地一个标签都没有，先打标签："
        echo "  bash tools/release.sh v0.1.0 \"说明\""
        exit 1
    fi
    REMOTE_TAGS="$(git ls-remote --tags origin 2>/dev/null \
        | sed 's#.*refs/tags/##' | grep -v '\^{}$' || true)"

    MISSING=""
    for t in $LOCAL_TAGS; do
        printf '%s\n' "$REMOTE_TAGS" | grep -qx "$t" || MISSING="$MISSING $t"
    done

    if [ -z "$MISSING" ]; then
        echo "✓ 本地标签都已在远程，无需补推"
        exit 0
    fi

    echo "→ 待补推的标签：$MISSING"
    # 逐个显式推（一次推多个也行，但逐个推失败时能看清是哪个）
    for t in $MISSING; do
        if git push origin "refs/tags/$t"; then
            echo "  ✓ $t"
        else
            echo "  ✗ $t 推送失败（认证/网络）"
            exit 1
        fi
    done

    echo
    echo "✓ 完成，回读远程确认："
    git ls-remote --tags origin 2>/dev/null \
        | sed 's#.*refs/tags/##' | grep -v '\^{}$' | sed 's/^/  /'
    exit 0
fi

TAG="${1:-}"
MSG="${2:-}"

if [ -z "$TAG" ]; then
    echo "用法: bash tools/release.sh <标签名> [说明]"
    echo "例:   bash tools/release.sh v0.2.1 \"修复：中止后卡执行中\""
    echo "      bash tools/release.sh --push-all     # 把本地标签全补推到远程"
    echo "      bash tools/release.sh --list         # 本地/远程标签对照"
    exit 1
fi

# 标签名基本校验：v 开头 + 数字，避免打成 "最终版" 这种排序会乱的名字
case "$TAG" in
    v[0-9]*) : ;;
    *) echo "✗ 标签名建议用 v主.次.修 形式（如 v0.2.1），当前是：$TAG"; exit 1 ;;
esac

# 已存在的标签不重复打（避免悄悄改掉历史里程碑），但**补推**是允许的：
# 本地有、远程没有 = 上次没推成功，这里直接推上去，不用删了重打。
TAG_EXISTS=0
if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
    TAG_EXISTS=1
    if git ls-remote --tags origin "refs/tags/$TAG" 2>/dev/null | grep -q .; then
        echo "✓ $TAG 本地和远程都已存在，无需操作"
        exit 0
    fi
    echo "→ $TAG 本地已存在但远程没有（上次没推成功），直接补推"
fi

# 工作区必须干净，否则标签指到的内容和你以为的不一样
if [ -n "$(git status --porcelain)" ]; then
    echo "✗ 工作区有未提交改动，先 commit 再打标签："
    git status --short
    exit 1
fi

if [ -z "$MSG" ]; then
    MSG="$(git log -1 --pretty=%s)"
fi

# 未推的提交先提示（标签指向的提交可能在远程不存在 → GitHub 会显示标签但点不开）
AHEAD="$(git rev-list --count "@{u}..HEAD" 2>/dev/null || echo 0)"
if [ "$AHEAD" != "0" ]; then
    echo "→ 先把 $AHEAD 个未推送的提交推上去"
    git push || { echo "✗ push 失败"; exit 1; }
fi

if [ "$TAG_EXISTS" = "0" ]; then
    echo "→ 打 annotated 标签 $TAG → $(git rev-parse --short HEAD)"
    git tag -a "$TAG" -m "$MSG" || exit 1
fi

# ★ 关键一步：显式推这个标签。不用 --follow-tags（它只管 annotated 且条件苛刻）
echo "→ 推送标签 $TAG"
if ! git push origin "refs/tags/$TAG"; then
    echo "✗ 标签推送失败（常见原因：没登录 GitHub / 网络）。"
    echo "  标签已在本地打好，稍后补推即可：git push origin $TAG"
    exit 1
fi

# 回读校验：不看本地，看**远程**有没有
if git ls-remote --tags origin "refs/tags/$TAG" 2>/dev/null | grep -q .; then
    echo "✓ 完成：$TAG 已在 GitHub 上"
    echo "  查看：https://github.com/$(git remote get-url origin \
        | sed -e 's#.*github.com[:/]##' -e 's#\.git$##')/tags"
else
    echo "✗ 推送命令返回成功，但远程查不到 $TAG。请手动确认。"
    exit 1
fi

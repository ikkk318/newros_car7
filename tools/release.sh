#!/usr/bin/env bash
# =============================================================================
# 打标签并推到 GitHub —— 一条命令搞定，顺带解决"推不上去"
#
# 为什么需要这个脚本：
#   1. `git push` 默认**不传标签**。标签要显式推，否则 GitHub 的 Tags 页面
#      永远是空的（本项目已经踩过一次：commit 全上去了，tags API 返回 []）。
#   2. `--follow-tags` 不可靠：它只处理 annotated 标签，而且**只有真的在推提交时
#      才顺带推标签**。main 已经是最新时它什么都不推 → 标签永远留在本地。
#   3. HTTPS 到 github.com:443 在部分网络下是**完全不通**的（本项目实测：443
#      三次全超时，22 三次全通）。这时候必须换通道，光重试 HTTPS 没用。
#
# 所以最稳的是：先测通道（--doctor），挑通的走，然后显式 `git push origin <tag>`。
#
# 用法：
#   bash tools/release.sh --doctor                        # 诊断：通道/凭据/标签，只读不动
#   bash tools/release.sh --push-all                      # 本地标签全补推（自动选通道）
#   bash tools/release.sh --list                          # 只看本地/远程标签对照
#   bash tools/release.sh v0.3.0 "能跑通一次搬运"          # 给当前 HEAD 打标签并推送
#   bash tools/release.sh v0.3.0                          # 说明省略则用最近一次提交标题
#   bash tools/release.sh --use-ssh                       # 把 origin 永久改成 SSH
#   bash tools/release.sh --use-https                     # 改回 HTTPS（必要时自动配代理）
# =============================================================================
set -uo pipefail

cd "$(dirname "$0")/.." || exit 1

REPO_SLUG="$(git remote get-url origin 2>/dev/null \
    | sed -e 's#.*github\.com[:/]##' -e 's#\.git$##')"
REPO_URL_HTTPS="https://github.com/${REPO_SLUG}.git"
REPO_URL_SSH="git@github.com:${REPO_SLUG}.git"

# 常见代理端口。18397/7897 是 Clash Verge，7890 是 Clash 内核默认，
# 1080/10809 是 v2ray。别只认一个 —— 本项目 7897 的 TLS 层就是坏的。
PROXY_CANDIDATES="7890 7897 1080 10809 8080"

# ── 小工具 ────────────────────────────────────────────────────────────────────

# 探测 TCP 端口是否可连（不依赖 nc，用 bash 自带 /dev/tcp）
# $1=host $2=port $3=超时秒
tcp_ok() {
    timeout "${3:-6}" bash -c "exec 3<>/dev/tcp/$1/$2" 2>/dev/null
}

# 探测某个 http 代理能否真正翻到 GitHub（只看 HTTP 状态码，不看内容）
# 注意：必须真的发一个 HTTPS 请求。只测端口通是没用的 —— 7897 端口在监听，
# 但经过它的 TLS 握手会 eof，端口通 ≠ 能用。
proxy_ok() {
    local p="$1"
    [ "$(timeout 12 curl -sS -x "http://127.0.0.1:$p" -o /dev/null \
        -w '%{http_code}' https://api.github.com 2>/dev/null)" = "200" ]
}

# 探测 443 直连能否真的建立 TLS。
# ★ 这里不能用 tcp_ok：本项目的网络是**间歇性**的，TCP 三次握手常常能过，
#   但 TLS 握手和真正传输会被掐断（报 GnuTLS / unexpected eof）。
#   只看端口通会得出"网络没问题"的错误结论，然后 push 卡 135 秒再超时。
https_direct_ok() {
    [ "$(timeout 15 curl -sS --noproxy '*' -o /dev/null \
        -w '%{http_code}' https://api.github.com 2>/dev/null)" = "200" ]
}

# 探测 SSH 到 GitHub 是否**认证成功**（不只是端口通）
# 成功时 GitHub 返回 "Hi <user>! You've successfully authenticated"，
# 且 SSH 退出码是 1（GitHub 不提供 shell，这是正常的）。
ssh_auth_ok() {
    local out
    out="$(timeout 20 ssh -T \
        -o BatchMode=yes -o ConnectTimeout=10 \
        -o StrictHostKeyChecking=accept-new \
        git@github.com 2>&1)"
    printf '%s' "$out" | grep -q "successfully authenticated"
}

# 探测 SSH 端口是否可连（区分"网络不通"和"公钥没注册"）
ssh_port_ok() { tcp_ok github.com 22 6; }

# 找第一个能用的代理端口，输出端口号（找不到输出空）
find_proxy() {
    for p in $PROXY_CANDIDATES; do
        if proxy_ok "$p"; then echo "$p"; return 0; fi
    done
    return 1
}

# 当前 origin 用的是 SSH 还是 HTTPS
remote_scheme() {
    case "$(git remote get-url origin 2>/dev/null)" in
        git@*|ssh://*) echo ssh ;;
        *) echo https ;;
    esac
}

# ── 诊断模式：只读，不改任何东西 ──────────────────────────────────────────────
if [ "${1:-}" = "--doctor" ]; then
    echo "═══ 1. 远端地址 ═══"
    echo "  origin = $(git remote get-url origin 2>/dev/null || echo '未设置')"
    echo "  通道   = $(remote_scheme)"

    echo
    echo "═══ 2. 网络通道 ═══"
    if https_direct_ok; then
        echo "  HTTPS 443 直连   ✓ TLS 通"
    elif tcp_ok github.com 443 6; then
        echo "  HTTPS 443 直连   ⚠ 端口能连但 TLS 断（间歇性，push 大概率卡住）"
    else
        echo "  HTTPS 443 直连   ✗ 不通（不挂代理时 git push 必然超时）"
    fi
    if ssh_port_ok; then
        echo "  SSH   22  直连   ✓ 通（走这条路不需要任何代理）"
    else
        echo "  SSH   22  直连   ✗ 不通"
    fi
    if P="$(find_proxy)"; then
        echo "  本地代理 $P      ✓ 可用"
    else
        echo "  本地代理         ✗ 候选端口（$PROXY_CANDIDATES）都不可用"
    fi

    echo
    echo "═══ 3. 凭据 ═══"
    if ssh_auth_ok; then
        echo "  SSH 公钥         ✓ 已被 GitHub 接受"
    elif ssh_port_ok; then
        echo "  SSH 公钥         ✗ 端口通但公钥未注册（把下面这行加到 GitHub Settings → SSH keys）"
        # 2>/dev/null：万一公钥文件读不到，别让 sed 的报错盖住真正的结论
        [ -f "$HOME/.ssh/id_ed25519.pub" ] && sed 's/^/                   /' "$HOME/.ssh/id_ed25519.pub" 2>/dev/null
    else
        echo "  SSH 公钥         — 端口不通，测不了"
    fi
    if [ -f "$HOME/.git-credentials" ] || git config --get credential.helper >/dev/null 2>&1; then
        echo "  HTTPS 凭据       ✓ 有 credential helper"
    else
        echo "  HTTPS 凭据       ✗ 没有（HTTPS 推送会要用户名/密码）"
    fi

    echo
    echo "═══ 4. 标签 ═══"
    echo "  本地：$(git tag -l | tr '\n' ' ')"
    if REMOTE_TAGS="$(git ls-remote --tags origin 2>/dev/null)"; then
        R="$(printf '%s\n' "$REMOTE_TAGS" | sed 's#.*refs/tags/##' | grep -v '\^{}$' | tr '\n' ' ')"
        echo "  远程：${R:-（空 —— GitHub 上一个标签都没有）}"
    else
        echo "  远程：（取不到，通道不通）"
    fi

    echo
    echo "═══ 5. 未推送的提交 ═══"
    git log --oneline '@{u}..HEAD' 2>/dev/null | sed 's/^/  /' || echo "  （取不到跟踪分支）"
    echo
    echo "结论：网络通 + 凭据 ✓ 之后跑  bash tools/release.sh --push-all"
    exit 0
fi

# ── 切换远端通道 ──────────────────────────────────────────────────────────────
if [ "${1:-}" = "--use-ssh" ]; then
    git remote set-url origin "$REPO_URL_SSH"
    echo "✓ origin → $REPO_URL_SSH"
    echo "  这一步只是换地址，公钥还得在 GitHub 上注册过。"
    ssh_auth_ok && echo "  ✓ SSH 认证正常" || echo "  ✗ SSH 认证失败，先跑 --doctor 看原因"
    exit 0
fi

if [ "${1:-}" = "--use-https" ]; then
    git remote set-url origin "$REPO_URL_HTTPS"
    echo "✓ origin → $REPO_URL_HTTPS"
    # HTTPS 直连大概率不通，有可用代理就顺手把它绑到这个仓库上
    if tcp_ok github.com 443 6; then
        git config --local --unset http.proxy 2>/dev/null
        echo "  ✓ 443 直连可用，不需要代理"
    elif P="$(find_proxy)"; then
        git config --local http.proxy "http://127.0.0.1:$P"
        echo "  ✓ 443 不通，已给本仓库配代理 127.0.0.1:$P（git config --local http.proxy）"
    else
        echo "  ✗ 443 不通且没有可用代理，HTTPS 推不上去。建议改用 --use-ssh"
    fi
    exit 0
fi

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
        echo "  （取不到，检查网络/代理，或用 --doctor 诊断）"
    fi
    echo
    echo "提示：本地有、远程没有 → 就是没推上去。用本脚本或"
    echo "      git push origin <标签名> 补推。"
    exit 0
fi

# ── 关键：推送前确保通道可用 ──────────────────────────────────────────────────
# 这是本脚本存在的主要理由。用户遇到的报错有两种，长得完全不一样但根因相同：
#   GnuTLS recv error (-110): The TLS connection was non-properly terminated.
#   Failed to connect to github.com port 443 after 134998 ms: 连接超时
# 前者是代理的 TLS 被掐断，后者是压根没走代理。两种情况都不该靠"再试一次"。
ensure_channel() {
    local scheme; scheme="$(remote_scheme)"

    if [ "$scheme" = "ssh" ]; then
        if ssh_auth_ok; then
            echo "  ✓ 通道：SSH（不需要代理）"
            return 0
        fi
        echo "  ✗ SSH 认证失败。跑 bash tools/release.sh --doctor 看是公钥没注册还是端口不通"
        return 1
    fi

    # HTTPS：直连（含 TLS）优先，不行就找代理，再不行建议换 SSH
    if https_direct_ok; then
        git config --local --unset http.proxy 2>/dev/null
        echo "  ✓ 通道：HTTPS 直连"
        return 0
    fi
    echo "  → 443 直连的 TLS 不通，找本地代理…"
    local p
    if p="$(find_proxy)"; then
        git config --local http.proxy "http://127.0.0.1:$p"
        echo "  ✓ 通道：HTTPS 经代理 127.0.0.1:$p（已写入本仓库配置）"
        return 0
    fi
    echo "  ✗ 443 不通、也没有可用代理。"
    echo "    最省事的解法是改用 SSH（22 端口通常不受影响）："
    echo "      bash tools/release.sh --use-ssh"
    return 1
}

# 带自动换通道重试的 push。
# 本项目的网络时通时断，单次失败不代表没救 —— 依次尝试
# ① 当前配置 ② 直连 ③ 各代理 ④ SSH（若公钥已注册），任一成功即返回。
# 用法：push_retry <refspec> [refspec...]
push_retry() {
    local refspecs=("$@")

    echo "→ 推送：${refspecs[*]}"

    # ① 当前配置直接试一次
    local out rc
    out="$(git push origin "${refspecs[@]}" 2>&1)"; rc=$?
    if [ $rc -eq 0 ]; then
        printf '%s\n' "$out"
        return 0
    fi
    printf '%s\n' "$out" | tail -3

    echo "  ⚠ 当前通道失败，轮换候选通道重试…"

    # ② 直连（清掉 proxy）
    if https_direct_ok; then
        echo "  → 试 HTTPS 直连"
        git config --local --unset http.proxy 2>/dev/null
        if out="$(git push origin "${refspecs[@]}" 2>&1)"; then
            echo "  ✓ 直连成功"; return 0
        fi
        printf '%s\n' "$out" | tail -2
    fi

    # ③ 逐个代理
    local p
    for p in $PROXY_CANDIDATES; do
        proxy_ok "$p" || continue
        echo "  → 试代理 127.0.0.1:$p"
        git config --local http.proxy "http://127.0.0.1:$p"
        if out="$(git push origin "${refspecs[@]}" 2>&1)"; then
            echo "  ✓ 代理 $p 成功"; return 0
        fi
        printf '%s\n' "$out" | tail -2
    done

    # ④ SSH 兜底（只在公钥已注册时才有意义；不改 origin，用一次性 URL）
    if ssh_auth_ok; then
        echo "  → 试 SSH 通道"
        if out="$(git push "$REPO_URL_SSH" "${refspecs[@]}" 2>&1)"; then
            echo "  ✓ SSH 成功"; return 0
        fi
        printf '%s\n' "$out" | tail -2
    fi

    echo "  ✗ 所有通道都失败。跑 bash tools/release.sh --doctor 定位"
    return 1
}

# ── 补推模式：把本地有、远程没有的标签全部推上去 ──────────────────────────────
if [ "${1:-}" = "--push-all" ]; then
    LOCAL_TAGS="$(git tag -l)"
    if [ -z "$LOCAL_TAGS" ]; then
        echo "本地一个标签都没有，先打标签："
        echo "  bash tools/release.sh v0.1.0 \"说明\""
        exit 1
    fi

    echo "→ 检查通道"
    ensure_channel || exit 1

    REMOTE_TAGS="$(git ls-remote --tags origin 2>/dev/null \
        | sed 's#.*refs/tags/##' | grep -v '\^{}$' || true)"

    MISSING=""
    for t in $LOCAL_TAGS; do
        printf '%s\n' "$REMOTE_TAGS" | grep -qx "$t" || MISSING="$MISSING $t"
    done

    # 除了标签，未推送的提交也一起推 —— 标签指向的提交远程不存在时，
    # GitHub 会出现"标签在、点开 404"的诡异状态。
    if [ -n "$(git log --oneline '@{u}..HEAD' 2>/dev/null)" ]; then
        echo "→ 先推未推送的提交"
        push_retry refs/heads/main:refs/heads/main || { echo "✗ 分支推送失败"; exit 1; }
    fi

    if [ -z "$MISSING" ]; then
        echo "✓ 本地标签都已在远程，无需补推"
        exit 0
    fi

    echo "→ 待补推的标签：$MISSING"
    # 逐个显式推（一次推多个也行，但逐个推失败时能看清是哪个）
    for t in $MISSING; do
        if push_retry "refs/tags/$t:refs/tags/$t"; then
            echo "  ✓ $t"
        else
            echo "  ✗ $t 推送失败"
            echo "    先跑 bash tools/release.sh --doctor 定位是网络还是凭据"
            exit 1
        fi
    done

    echo
    echo "✓ 完成，回读远程确认："
    git ls-remote --tags origin 2>/dev/null \
        | sed 's#.*refs/tags/##' | grep -v '\^{}$' | sed 's/^/  /'
    echo
    echo "  查看：https://github.com/${REPO_SLUG}/tags"
    exit 0
fi

TAG="${1:-}"
MSG="${2:-}"

if [ -z "$TAG" ]; then
    echo "用法: bash tools/release.sh <标签名> [说明]"
    echo "例:   bash tools/release.sh --doctor          # 先诊断网络/凭据"
    echo "      bash tools/release.sh --push-all        # 把本地标签全补推到远程"
    echo "      bash tools/release.sh --list            # 本地/远程标签对照"
    echo "      bash tools/release.sh --use-ssh         # origin 改用 SSH"
    echo "      bash tools/release.sh v0.3.0 \"能跑通一次搬运\""
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

echo "→ 检查通道"
ensure_channel || exit 1

# 未推的提交先提示（标签指向的提交可能在远程不存在 → GitHub 会显示标签但点不开）
AHEAD="$(git rev-list --count "@{u}..HEAD" 2>/dev/null || echo 0)"
if [ "$AHEAD" != "0" ]; then
    echo "→ 先把 $AHEAD 个未推送的提交推上去"
    push_retry refs/heads/main:refs/heads/main || { echo "✗ push 失败"; exit 1; }
fi

if [ "$TAG_EXISTS" = "0" ]; then
    echo "→ 打 annotated 标签 $TAG → $(git rev-parse --short HEAD)"
    git tag -a "$TAG" -m "$MSG" || exit 1
fi

# ★ 关键一步：显式推这个标签。不用 --follow-tags —— 它在 main 已是最新时
#   根本不会触发，标签会永远留在本地。这个坑本项目踩过一次。
echo "→ 推送标签 $TAG"
if ! push_retry "refs/tags/$TAG:refs/tags/$TAG"; then
    echo "✗ 标签推送失败。标签已在本地打好，定位原因后补推："
    echo "    bash tools/release.sh --doctor"
    echo "    bash tools/release.sh --push-all"
    exit 1
fi

# 回读校验：不看本地，看**远程**有没有
if git ls-remote --tags origin "refs/tags/$TAG" 2>/dev/null | grep -q .; then
    echo "✓ 完成：$TAG 已在 GitHub 上"
    echo "  查看：https://github.com/${REPO_SLUG}/tags"
else
    echo "✗ 推送命令返回成功，但远程查不到 $TAG。请手动确认。"
    exit 1
fi

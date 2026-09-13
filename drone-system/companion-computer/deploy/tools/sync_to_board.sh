#!/bin/bash
# 将本机最新代码同步到板子 (ubuntu-pi)
#
# 用法:
#   ./tools/sync_to_board.sh                   # 同步 basic 版本
#   ./tools/sync_to_board.sh basic_radar       # 同步 basic_radar 版本
#   ./tools/sync_to_board.sh competition_2026  # 同步备赛版
#
# 板子路径: /home/sunrise/Desktop/FJJ/<version>/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BOARD_HOST="ubuntu-pi"
BOARD_BASE="/home/sunrise/Desktop/FJJ"

echo "=============================================="
echo "  同步代码到板子 ($BOARD_HOST)"
echo "=============================================="
echo ""

# 检查板子连接
ssh -o ConnectTimeout=5 -o BatchMode=yes "$BOARD_HOST" "echo 连通性检查通过" 2>/dev/null || {
    echo "❌ 无法连接 $BOARD_HOST，请检查网络和 SSH 配置"
    exit 1
}
echo "✅ 板子连接正常"
echo ""

VERSION="${1:-basic}"
SRC_DIR="$PROJECT_DIR/drone_control/$VERSION"

if [ ! -d "$SRC_DIR" ]; then
    echo "❌ 本地目录不存在: $SRC_DIR"
    exit 1
fi

echo "── 同步 $VERSION ──"
echo "  本地: $SRC_DIR/"
echo "  板子: $BOARD_BASE/$VERSION/"
echo ""

# 用 tar 流式传输，排除不需要的文件
# 注意: 在板子上解压到 BOARD_BASE，所以 tar 的路径相对于 drone_control/
cd "$PROJECT_DIR/drone_control"
tar cf - \
    --exclude="__pycache__" \
    --exclude=".pytest_cache" \
    --exclude="*.pyc" \
    --exclude="flight_data.jsonl" \
    --exclude="fc_log.log" \
    --exclude="flight_data_*.jsonl.bak" \
    "$VERSION" | \
ssh "$BOARD_HOST" "tar xf - -C $BOARD_BASE"

echo "✅ $VERSION 同步完成"

# 同步 tools/（拉日志、分析等脚本）
echo ""
echo "── 同步 tools/ ──"
cd "$PROJECT_DIR"
tar cf - \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    "tools" | \
ssh "$BOARD_HOST" "tar xf - -C $BOARD_BASE"
echo "✅ tools 同步完成"

echo ""
echo "=============================================="
echo "  同步完成"
echo "=============================================="
echo ""
echo "板子上测试文件:"
ssh "$BOARD_HOST" "ls -la $BOARD_BASE/$VERSION/router_tests/" 2>/dev/null || echo "  (无 router_tests/ 目录)"

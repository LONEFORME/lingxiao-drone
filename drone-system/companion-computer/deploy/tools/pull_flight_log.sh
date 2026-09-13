#!/bin/bash
# 从板子拉取飞行日志并归档到本地
#
# 用法:
#   ./tools/pull_flight_log.sh                         # 拉取 basic 版本
#   ./tools/pull_flight_log.sh basic_radar             # 拉取 basic_radar 版本
#   ./tools/pull_flight_log.sh competition_2026        # 拉取备赛版
#
# 数据只保存在本地，板子上不留副本
# 本地归档: drone_control/tools/data_archive/test_data_YYYYMMDD/

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

BOARD_HOST="ubuntu-pi"
BOARD_BASE="/home/sunrise/Desktop/FJJ"
VERSION="${1:-basic}"
DATE_TAG=$(date +%Y%m%d)

ARCHIVE_DIR="$PROJECT_DIR/drone_control/tools/data_archive"
ARCHIVE_SUBDIR="test_data_${DATE_TAG}"
LOCAL_PATH="$ARCHIVE_DIR/$ARCHIVE_SUBDIR"

echo "=== 拉取 $VERSION 飞行日志 === "

BOARD_DIR="$BOARD_BASE/$VERSION"

# 检查板子上有没有数据
LOG_EXISTS=$(ssh "$BOARD_HOST" "test -f $BOARD_DIR/flight_data.jsonl && wc -c < $BOARD_DIR/flight_data.jsonl || echo 0" 2>/dev/null)
if [ "$LOG_EXISTS" = "0" ] || [ -z "$LOG_EXISTS" ]; then
    echo "⚠  板子上 $BOARD_DIR/flight_data.jsonl 不存在或为空"
    exit 0
fi

# 从 router.txt 提取路径概要作为文件名描述
DESC=$(ssh "$BOARD_HOST" "
    if [ -f $BOARD_DIR/router.txt ]; then
        awk -F, '{printf \"%.1f_%.1f_\", \$1, \$2}' $BOARD_DIR/router.txt | head -c 40
    else
        echo 'norouter'
    fi
" 2>/dev/null || echo "unknown")

# 拉取到本地归档
echo "  拉取中..."
mkdir -p "$LOCAL_PATH"

FILENAME="flight_data_${DESC}_${DATE_TAG}.jsonl"
N=1
while [ -f "$LOCAL_PATH/$FILENAME" ]; do
    N=$((N + 1))
    FILENAME="flight_data_${DESC}_${DATE_TAG}_v${N}.jsonl"
done

scp "$BOARD_HOST:$BOARD_DIR/flight_data.jsonl" "$LOCAL_PATH/$FILENAME"
scp "$BOARD_HOST:$BOARD_DIR/router.txt" "$LOCAL_PATH/router_${DATE_TAG}.txt" 2>/dev/null || true

echo "  📁 $LOCAL_PATH/$FILENAME"

# 清空板子工作日志（为下次测试准备）
echo "  清空板子工作日志..."
ssh "$BOARD_HOST" "truncate -s 0 $BOARD_DIR/flight_data.jsonl" 2>/dev/null

echo ""
echo "✅ 完成"
echo ""
echo "分析:"
echo "  python3 tools/flight_log_analyzer.py \"$LOCAL_PATH/$FILENAME\""
echo ""
echo "当前归档:"
ls -1 "$LOCAL_PATH/" 2>/dev/null

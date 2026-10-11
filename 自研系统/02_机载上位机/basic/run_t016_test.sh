#!/bin/bash
# T-016 Basic 版本完整系统测试 — 一键测试脚本（板载版）
#
# 用法:
#   ./run_t016_test.sh long              # 测试A：长路径 (2m×2m)
#   ./run_t016_test.sh short             # 测试B：短路径 (0.5m×0.5m)
#   ./run_t016_test.sh height            # 测试C：高度变化
#   ./run_t016_test.sh                   # 显示帮助
#
# 流程（本机运行）:
#   1. 将测试 router 推送到板子
#   2. 提示用户在板子上按一键起飞按钮
#   3. 飞行完成后回车确认
#   4. 拉取板子上的飞行日志到本地归档
#   5. 分析本次飞行日志
#   6. 还原板子上的 router.txt

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
TOOLS_DIR="$PROJECT_DIR/tools"

BOARD_HOST="ubuntu-pi"
BOARD_BASE="/home/sunrise/Desktop/FJJ"

# ----- 帮助 -----
usage() {
    echo "T-016 Basic 版本完整系统测试"
    echo ""
    echo "用法（在本机运行）："
    echo "  $0 long      测试A：长路径 (2m×2m 矩形，评估 T265 漂移累积)"
    echo "  $0 short     测试B：短路径 (0.5m 正方形，评估低空精确定位)"
    echo "  $0 height    测试C：高度变化 (0.5/1.0/1.5m，评估垂直通道)"
    echo ""
    echo "脚本会自动将测试 router 推送到板子，飞行后拉取日志并分析。"
    echo "在执行飞行前，建议先运行 ./tools/sync_to_board.sh basic 同步最新代码。"
    exit 0
}

# ----- 检查工具脚本 -----
check_tools() {
    if [ ! -f "$TOOLS_DIR/pull_flight_log.sh" ]; then
        echo "❌ 未找到 pull_flight_log.sh"
        echo "   预期路径: $TOOLS_DIR/pull_flight_log.sh"
        exit 1
    fi
}

# ----- 检查板子连接 -----
check_board() {
    echo "检查板子连接..."
    if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$BOARD_HOST" "echo ok" 2>/dev/null; then
        echo "❌ 无法连接 $BOARD_HOST"
        echo "   请先确认板子已上电且网络可达"
        exit 1
    fi
    echo "✅ 板子连接正常"
}

# ----- 选择测试 -----
case "${1:-help}" in
    long)
        TEST_NAME="A-长路径"
        TEST_FILE="router_tests/router_t016_long.txt"
        TEST_DESC="long"
        ;;
    short)
        TEST_NAME="B-短路径"
        TEST_FILE="router_tests/router_t016_short.txt"
        TEST_DESC="short"
        ;;
    height)
        TEST_NAME="C-高度变化"
        TEST_FILE="router_tests/router_t016_height.txt"
        TEST_DESC="height"
        ;;
    *)
        usage
        ;;
esac

check_tools

echo "=============================================="
echo "  T-016 测试 ${TEST_NAME}"
echo "=============================================="
echo ""

# ----- 检查本地测试文件 -----
if [ ! -f "$SCRIPT_DIR/$TEST_FILE" ]; then
    echo "❌ 测试文件不存在: $SCRIPT_DIR/$TEST_FILE"
    echo "   请先运行 ./tools/sync_to_board.sh basic 同步最新代码"
    exit 1
fi

check_board

# ----- 1. 备份板子上的 router.txt -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 1：备份板子上的 router.txt"
echo "──────────────────────────────────────"
BACKUP_EXISTS=$(ssh "$BOARD_HOST" "test -f $BOARD_BASE/basic/router.txt && echo yes || echo no")
if [ "$BACKUP_EXISTS" = "yes" ]; then
    ssh "$BOARD_HOST" "cp $BOARD_BASE/basic/router.txt $BOARD_BASE/basic/router_backup.txt"
    echo "✅ 已备份板子上的 router.txt → router_backup.txt"
else
    echo "ℹ️  板子上无现有 router.txt，跳过备份"
fi

# ----- 2. 推送测试 router 到板子 -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 2：推送测试航路到板子"
echo "──────────────────────────────────────"
scp "$SCRIPT_DIR/$TEST_FILE" "$BOARD_HOST:$BOARD_BASE/basic/router.txt"
echo "✅ 已推送测试航路到板子: $TEST_FILE"
echo ""
echo "📋 航点列表:"
grep -v '^#' "$SCRIPT_DIR/$TEST_FILE" | grep -v '^$' | while IFS=',' read -r x y z; do
    printf "    (%+.1f, %+.1f, %.2f)\n" "$x" "$y" "$z"
done

# ----- 3. 提示飞行 -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 3：执行飞行"
echo "──────────────────────────────────────"
echo ""
echo "✅ 测试航路已推送到板子"
echo ""
echo "请在板子上操作："
echo "  1. cd ~/Desktop/FJJ/basic"
echo "  2. python3 main.py"
echo "  3. 按一键起飞按钮 → 自动执行测试航路"
echo ""
echo "飞行完成后，在此终端按回车继续..."
read -r

# ----- 4. 拉取日志 -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 4：拉取飞行日志"
echo "──────────────────────────────────────"
echo ""
bash "$TOOLS_DIR/pull_flight_log.sh" basic

# ----- 5. 还原板子上的 router.txt -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 5：还原板子上的 router.txt"
echo "──────────────────────────────────────"
if [ "$BACKUP_EXISTS" = "yes" ]; then
    ssh "$BOARD_HOST" "mv $BOARD_BASE/basic/router_backup.txt $BOARD_BASE/basic/router.txt"
    echo "✅ 已还原板子上的 router.txt"
else
    ssh "$BOARD_HOST" "rm -f $BOARD_BASE/basic/router.txt"
    echo "✅ 已清理板子上的测试 router.txt（之前无备份）"
fi

# ----- 6. 分析日志 -----
echo ""
echo "──────────────────────────────────────"
echo "  步骤 6：分析本次飞行日志"
echo "──────────────────────────────────────"
echo ""
ARCHIVE_DIR="$PROJECT_DIR/drone_control/tools/data_archive"
if [ -d "$ARCHIVE_DIR" ]; then
    LATEST_DIR=$(ls -td "$ARCHIVE_DIR/test_data_"* 2>/dev/null | head -1)
    if [ -n "$LATEST_DIR" ]; then
        LATEST_LOG=$(ls -t "$LATEST_DIR"/*.jsonl 2>/dev/null | head -1)
        if [ -n "$LATEST_LOG" ]; then
            echo "最新日志: $LATEST_LOG"
            echo ""
            python3 "$TOOLS_DIR/flight_log_analyzer.py" "$LATEST_LOG"
        else
            echo "⚠  归档目录中无 .jsonl 文件"
            echo "   可手动分析: python3 $TOOLS_DIR/flight_log_analyzer.py <文件路径>"
        fi
    else
        echo "⚠  未找到归档目录"
        echo "   可手动分析: python3 $TOOLS_DIR/flight_log_analyzer.py <文件路径>"
    fi
fi

echo ""
echo "=============================================="
echo "  T-016 测试 ${TEST_NAME} 完成"
echo "=============================================="
echo ""
echo "通过标准速查："
echo "  □ 回到原点/各角 XY 偏差 < 10~15cm"
echo "  □ 降落锁桨 (unlock_sta=0, motor_pwm_mask=0)"
echo "  □ 航向保持无故障 (heading_fault_reason=null)"
echo "  □ Yaw 全程漂移 < 5°"
echo ""
echo "详细标准见 docs/TODO.md"

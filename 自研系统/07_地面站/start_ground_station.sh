#!/usr/bin/env bash
# N100 GNOME 桌面启动器：可由登录后的自动启动项或 SSH 远程调用。
set -eu

APP_DIR="$HOME/Desktop/26省赛"
LOG_FILE="$APP_DIR/ground_station.log"

# 双击桌面图标时先关闭同一目录下的旧版本，确保重新加载刚更新的 main.py。
# 方括号避免 pkill 匹配到本启动脚本自身的命令行。
if pgrep -f '[2]6省赛/.venv/bin/python /home/n100/Desktop/26省赛/main.py' >/dev/null; then
    pkill -f '[2]6省赛/.venv/bin/python /home/n100/Desktop/26省赛/main.py'
    sleep 1
fi

# 登录后的 GNOME 会话会提供 DISPLAY；SSH 远程启动时回退到本机主显示器。
export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
    GDM_XAUTH="/run/user/$(id -u)/gdm/Xauthority"
    if [ -f "$GDM_XAUTH" ]; then
        export XAUTHORITY="$GDM_XAUTH"
    elif [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
fi

exec "$APP_DIR/.venv/bin/python" "$APP_DIR/main.py" >> "$LOG_FILE" 2>&1

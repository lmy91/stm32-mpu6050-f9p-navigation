#!/bin/sh

SERVICE=gnss-imu-logger.service

restore_terminal() {
    printf '\033[?7h'
}

stop_watching() {
    printf '\n'
    exit 0
}

trap stop_watching INT TERM HUP
trap restore_terminal EXIT

terminal_width() {
    width=$(tput cols 2>/dev/null || printf '120')
    case "$width" in
        ''|*[!0-9]*) width=120 ;;
    esac
    if [ "$width" -lt 40 ]; then
        width=40
    fi
    # Keep two columns unused so terminals that wrap at the last column do not
    # advance to another physical row.
    printf '%s' "$((width - 2))"
}

# A carriage return can only refresh one physical terminal row. Disable
# automatic wrapping as an additional guard, then truncate to the live width.
printf '\033[?7l'

while systemctl is-active --quiet "$SERVICE"; do
    status=$(journalctl -u "$SERVICE" -n 30 --no-pager -o cat 2>/dev/null |
        grep '^\[' | tail -n 1)
    if [ -z "$status" ]; then
        status="等待第一条采集状态..."
    fi
    width=$(terminal_width)
    status=$(printf '%s' "$status" | cut -c "1-$width")
    printf '\r\033[2K%-*s' "$width" "$status"
    sleep 1
done

printf '\r\033[2K采集服务未运行：%s\n' "$SERVICE"

#!/bin/sh

set -eu

SERVICE=gnss-imu-logger.service
DATA_ROOT=/home/lmy/stm32-mpu6050-f9p-navigation/data/decoded
EXPECTED_ROOT=/home/lmy/stm32-mpu6050-f9p-navigation/data/decoded

mkdir -p "$DATA_ROOT"
resolved=$(readlink -f "$DATA_ROOT")
if [ "$resolved" != "$EXPECTED_ROOT" ]; then
    echo "拒绝清理：数据目录不符合预期：$resolved" >&2
    exit 1
fi

list_sessions() {
    find "$DATA_ROOT" -regextype posix-extended -mindepth 1 -maxdepth 1 \
        -type d -regex '.*/[0-9]{14}(_[0-9]{2})?'
}

count=$(list_sessions | wc -l)
size=$(du -sh "$DATA_ROOT" | awk '{print $1}')

echo "待清理目录：$DATA_ROOT"
echo "时间戳会话：$count 个，占用：$size"

if [ "$count" -eq 0 ]; then
    echo "没有可清理的采集会话。"
    exit 0
fi

if [ "${1:-}" != "--yes" ]; then
    printf "确认永久删除全部采集会话？输入 yes："
    read -r answer
    if [ "$answer" != "yes" ]; then
        echo "已取消，未删除任何数据。"
        exit 0
    fi
fi

was_active=0
if systemctl is-active --quiet "$SERVICE"; then
    was_active=1
    sudo systemctl stop "$SERVICE"
fi

restore_service() {
    if [ "$was_active" -eq 1 ]; then
        sudo systemctl start "$SERVICE"
    fi
}
trap restore_service EXIT HUP INT TERM

list_sessions | while IFS= read -r session; do
    rm -rf -- "$session"
done

echo "采集会话已清理。"
if [ "$was_active" -eq 1 ]; then
    echo "采集服务正在恢复，将创建新的时间戳目录。"
fi


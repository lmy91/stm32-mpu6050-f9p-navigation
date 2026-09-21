#!/bin/sh

set -eu

SERVICE=gnss-imu-logger.service
CONTROL_FILE=/run/gnss-imu/recording.enabled

if ! systemctl is-active --quiet "$SERVICE"; then
    echo "定位服务未运行，请先执行：sudo systemctl start $SERVICE" >&2
    exit 1
fi

touch "$CONTROL_FILE"
echo "已请求开始采集；即将新建时间戳目录并保存 imu.csv、gnss.csv、rawx.csv、f9p.ubx。"

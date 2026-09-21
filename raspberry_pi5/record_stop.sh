#!/bin/sh

set -eu

CONTROL_FILE=/run/gnss-imu/recording.enabled

rm -f -- "$CONTROL_FILE"
echo "已请求停止采集；文件将立即刷新并关闭，实时定位服务继续运行。"

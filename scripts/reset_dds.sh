#!/usr/bin/env bash
set -o pipefail

ALIVE=$(pgrep -c -x gzserver)
ROS=$(pgrep -c -f "[r]os2 launch")
if [ "${ALIVE:-0}" != "0" ] || [ "${ROS:-0}" != "0" ]; then
  echo "[reset_dds] 还有 ROS/Gazebo 进程在跑（gzserver=$ALIVE ros2_launch=$ROS）"
  echo "            本脚本只清理孤儿锁文件，请先正常退出仿真（Ctrl+C，别用 kill -9）。"
  exit 1
fi

BEFORE=$(ls /dev/shm 2>/dev/null | grep -c "fastrtps" || true)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
AFTER=$(ls /dev/shm 2>/dev/null | grep -c "fastrtps" || true)
echo "[reset_dds] 清理前 fastrtps 相关文件 $BEFORE 个，清理后 $AFTER 个"

ros2 daemon stop --force >/dev/null 2>&1 || true
echo "[reset_dds] ros2 daemon 已重启（下一次 CLI 调用会自动拉起）"

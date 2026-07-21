#!/usr/bin/env bash

set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_humble_main/install/setup.bash
source /home/nvidia/patrol_ws/install/setup.bash
set -u

COMMAND="${1:-help}"

case "$COMMAND" in
  manual)
    echo "进入手动模式"
    echo "Q/W/E：左前/前进/右前"
    echo "A/S/D：左转/停止/右转"
    echo "Z/X/C：左后/后退/右后"
    echo "空格停止，英文句号退出"
    ros2 run patrol_teleop teleop_node
    ;;

  auto)
    ros2 service call \
      /patrol/set_control_mode \
      patrol_interfaces/srv/SetControlMode \
      "{mode: 2}"
    ;;

  stop)
    ros2 service call \
      /patrol/set_control_mode \
      patrol_interfaces/srv/SetControlMode \
      "{mode: 0}"
    ;;

  mission-start)
    ros2 service call \
      /patrol/set_control_mode \
      patrol_interfaces/srv/SetControlMode \
      "{mode: 2}"

    sleep 0.5

    ros2 service call \
      /patrol/mission/start \
      std_srvs/srv/Trigger \
      "{}"
    ;;

  mission-stop)
    ros2 service call \
      /patrol/mission/stop \
      std_srvs/srv/Trigger \
      "{}"

    sleep 0.5

    ros2 service call \
      /patrol/set_control_mode \
      patrol_interfaces/srv/SetControlMode \
      "{mode: 0}"
    ;;

  status)
    echo "===== 控制模式 ====="
    timeout 3 ros2 topic echo \
      /patrol/control_mode \
      --once || true

    echo "===== 任务状态 ====="
    timeout 3 ros2 topic echo \
      /patrol/mission/status \
      --once || true
    ;;

  *)
    echo "用法："
    echo "  $0 manual"
    echo "  $0 auto"
    echo "  $0 stop"
    echo "  $0 mission-start"
    echo "  $0 mission-stop"
    echo "  $0 status"
    ;;
esac

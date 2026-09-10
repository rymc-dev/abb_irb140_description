#!/usr/bin/env bash
# Build + run both containers without the compose plugin.
#   ./run.sh build     build both images
#   ./run.sh up        (re)start both containers
#   ./run.sh down      stop + remove both
#   ./run.sh logs      follow logs
set -euo pipefail
cd "$(dirname "$0")"

ROBOT_IP="${ROBOT_IP:-192.168.125.1}"
J23_COUPLED="${J23_COUPLED:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"   # match the host
SPEED_SCALE="${SPEED_SCALE:-2.0}"                               # 2.0 = double motion speed

case "${1:-up}" in
  build)
    docker build -f Dockerfile.ros1  -t abb-ros1:noetic  .
    docker build -f Dockerfile.relay -t abb-relay:jazzy   .
    ;;
  up)
    docker rm -f abb-ros1 abb-relay 2>/dev/null || true
    docker run -d --name abb-ros1 --network host --restart unless-stopped \
      -e ROBOT_IP="$ROBOT_IP" -e J23_COUPLED="$J23_COUPLED" \
      abb-ros1:noetic
    sleep 6
    docker run -d --name abb-relay --network host --ipc=host --restart unless-stopped \
      -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
      -e ROSBRIDGE_URL=ws://127.0.0.1:9090 -e SPEED_SCALE="$SPEED_SCALE" \
      -e FJT_ACTION_NAME="${FJT_ACTION_NAME:-/arm_controller/follow_joint_trajectory}" \
      -e ACTION_SPEED_SCALE="${ACTION_SPEED_SCALE:-1.0}" \
      abb-relay:jazzy
    echo "up. check:  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 topic echo /joint_states"
    ;;
  down)
    docker rm -f abb-ros1 abb-relay 2>/dev/null || true
    ;;
  logs)
    docker logs -f abb-relay &
    docker logs -f abb-ros1
    ;;
  *)
    echo "usage: $0 {build|up|down|logs}" >&2; exit 1;;
esac

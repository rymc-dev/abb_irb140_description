#!/usr/bin/env bash
# roscore + abb_driver (via /robot.launch) + rosbridge_server WebSocket on :9090
set -eo pipefail

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export J23_COUPLED="${J23_COUPLED:-false}"
export ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
: "${ROBOT_IP:?Set ROBOT_IP to the ABB IRC5 controller address (e.g. 192.168.125.1)}"

pids=()
cleanup() { echo "[entrypoint] shutting down..."; kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "[entrypoint] launching abb_driver -> robot_ip:=${ROBOT_IP} J23_coupled:=${J23_COUPLED}"
roslaunch /robot.launch robot_ip:="${ROBOT_IP}" J23_coupled:="${J23_COUPLED}" &
pids+=($!)

echo "[entrypoint] waiting for ROS 1 master"
until rostopic list >/dev/null 2>&1; do sleep 0.5; done

echo "[entrypoint] starting rosbridge_server on ws://0.0.0.0:${ROSBRIDGE_PORT}"
exec roslaunch rosbridge_server rosbridge_websocket.launch port:="${ROSBRIDGE_PORT}"

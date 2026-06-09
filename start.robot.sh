#!/bin/bash
# Real robot mode: no Gazebo, connects to physical TurtleBot3 on the network
# Requires: native Linux or native Docker (not Docker Desktop/WSL2)
# Robot must be on the same network and broadcasting ROS2 DDS
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_FILE="$SCRIPT_DIR/log/robot_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$SCRIPT_DIR/log"
echo "Logging to $LOG_FILE"

docker stop farming_tb3 2>/dev/null || true
kill $(lsof -ti:8080) 2>/dev/null || true

python3 -m http.server --directory "$SCRIPT_DIR/src/tb3_dashboard/web" 8080 &
HTTP_PID=$!
echo "Dashboard: http://localhost:8080"

# --net=host required so ROS2 DDS can discover the real robot on the network
# rosbridge on port 9090 will be accessible via host network
docker run --rm -i --name farming_tb3 \
  --user $(id -u):$(id -g) \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  -v "$SCRIPT_DIR":/ws \
  farming_turtlebot bash -c "
    cd /ws &&
    source /opt/ros/jazzy/setup.bash &&
    source /opt/turtlebot3_ws/install/setup.bash &&
    colcon build &&
    source install/setup.bash &&
    export TURTLEBOT3_MODEL=burger &&
    ros2 launch tb3_navigation_dt navigation_robot.launch.py
  " 2>&1 | tee "$LOG_FILE"

kill $HTTP_PID 2>/dev/null || true

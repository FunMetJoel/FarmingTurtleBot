#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_FILE="$SCRIPT_DIR/log/run_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to $LOG_FILE"

python3 -m http.server --directory "$SCRIPT_DIR/src/tb3_dashboard/web" 8080 &
HTTP_PID=$!
echo "Dashboard: http://localhost:8080 (PID $HTTP_PID)"

docker run --rm -i --name farming_tb3 \
  --user $(id -u):$(id -g) \
  -p 9090:9090 \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
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
    ros2 launch tb3_navigation_dt navigation_slam.launch.py
  " 2>&1 | tee "$LOG_FILE"

kill $HTTP_PID 2>/dev/null

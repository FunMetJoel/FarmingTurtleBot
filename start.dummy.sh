#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_FILE="$SCRIPT_DIR/log/dummy_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to $LOG_FILE"

python3 -m http.server --directory "$SCRIPT_DIR/src/tb3_dashboard/web" 8080 &
HTTP_PID=$!
echo "Dashboard: http://localhost:8080 (PID $HTTP_PID)"

docker run --rm -i --name farming_tb3 \
  --user $(id -u):$(id -g) \
  -p 9090:9090 \
  -v "$SCRIPT_DIR":/ws \
  farming_turtlebot bash -c "
    cd /ws &&
    source /opt/ros/jazzy/setup.bash &&
    source /opt/turtlebot3_ws/install/setup.bash &&
    colcon build --packages-select tb3_dashboard &&
    source install/setup.bash &&
    ros2 launch tb3_dashboard dashboard.launch.py
  " 2>&1 | tee "$LOG_FILE"

kill $HTTP_PID 2>/dev/null

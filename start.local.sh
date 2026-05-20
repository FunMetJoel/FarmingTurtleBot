#!/bin/bash
docker run --rm -it --name farming_tb3 \
  --user $(id -u):$(id -g) \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
  -v $(cd $(dirname "$0") && pwd):/ws \
  farming_turtlebot bash -c "
    cd /ws &&
    source /opt/ros/jazzy/setup.bash &&
    source /opt/turtlebot3_ws/install/setup.bash &&
    colcon build &&
    source install/setup.bash &&
    export TURTLEBOT3_MODEL=burger &&
    ros2 launch tb3_navigation_dt navigation_slam.launch.py
  "


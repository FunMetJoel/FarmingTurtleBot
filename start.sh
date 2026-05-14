#!/bin/bash
docker run --rm -it --name farming_tb3 \
  --user $(id -u):$(id -g) \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /mnt/wslg/.X11-unix:/tmp/.X11-unix \
  -v $(cd $(dirname "$0") && pwd):/ws \
  farming_turtlebot bash -c "
    cd /ws &&
    source /opt/ros/jazzy/setup.bash &&
    source /opt/turtlebot3_ws/install/setup.bash &&
    colcon build &&
    source install/setup.bash &&
    export TURTLEBOT3_MODEL=burger &&
    ros2 launch my_tb3_world new_world.launch.py
  "

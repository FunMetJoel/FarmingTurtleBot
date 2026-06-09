docker run --rm -it --name turtlebot3_container \
 -p 8080:8080 \
 --workdir /ws \
 -e DISPLAY=$DISPLAY \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v /home/c2irr10/turtlebot3_ws:/ws \
 --user $(id -u):$(id -g) \
 turtlebot3_ws \
 bash -c "source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash && source install/setup.bash && export TURTLEBOT3_MODEL=burger && ros2 launch my_tb3_world new_world.launch.py && exec bash"



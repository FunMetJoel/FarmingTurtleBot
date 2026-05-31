docker exec -it turtlebot3_container bash -c "\
cd /ws && \
source install/setup.bash && \
export TURTLEBOT3_MODEL=burger && \
exec bash"
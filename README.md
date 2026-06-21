# Farming Turtle Bot

We are *Team 9*.

Authors:

Joël Rodenbach - @FunMetJoel  
Samu Németh - @samunemeth  
Ignacy Świderski - @igifigi  
Can Erturk - @ErturkCan  
Marnix van den Bosch - @Marnix900
Frederic Cahn von Seelen - @frederic-cvs

This is a repository for keeping our code for the
CBL Autonomous Systems Twinning course.

Please keep commit messages concise and all lower case. Add extra detail to the
description if needed.

For instructions on demonstrating state synchronisation consult the [README of the `tb3_state_dt` package](./src/tb3_state_dt/README.md).

## Running in docker container (with simulated TurtleBot)
To run the code without having access to the lab laptop:
### 1. Run the docker container

1. Open WSL  
2. Start up docker desktop, or another way to run docker
3. Setup the docker container:  
   ```sh
   docker run --rm -it --name turtlebot3_container \
    -p 8080:8080 \
    --workdir /ws \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/c2irr10/turtlebot3_ws:/ws \
    --user $(id -u):$(id -g) \
    turtlebot3_ws \
    bash
   ```
4. Build and source  
   Inside the docker container, run:  
    ```bash
    source /opt/ros/jazzy/setup.bash 
    source /opt/turtlebot3_ws/install/setup.bash 
    rm -rf build/ install/ log/
    colcon build --symlink-install
    source install/setup.bash 
    export TURTLEBOT3_MODEL=burger 
    ```

5. Run simulation  
   Without the lab laptop, you need to start a gazebo simulation of the lab robot. To do this, run the following in the docker terminal
   ```bash
   ros2 launch my_tb3_world new_world.launch.py
   ```
   If this works correctly, a Gazebo window should pup up and look like this:
   ![Gazebo window](Gazebo.png)

6. Launch custom ROS2 nodes  
   To launch all build nodes, first open a new wsl terminal, and join the docker container
   ```
   docker exec -it turtlebot3_container bash -c "\
   cd /ws && \
   source install/setup.bash && \
   export TURTLEBOT3_MODEL=burger && \
   exec bash"
   ```

   Then, inside the docker terminal, run the master launch file
   ```
   ros2 launch my_tb3_world master.launch.py
   ```


## Running on lab laptop (with real TurtleBot)

1. SSH in to the TurtleBot:  
   Get the IP address thats on the TurtleBot
   ```sh
   ssh turtlebot@{IP_OF_TURTLEBOT}
   ```
2. Bringup turtlebot  
   In the SSH terminal, run
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
3. Clone repo  
   Make sure you got the latest version of the repository on the machine:
   ```
   git clone https://github.com/FunMetJoel/FarmingTurtleBot.git
   ```
4. Build and source  
   Open a new terminal on the lap laptop (so not SSH'st in to the turtlebot)   
    ```bash
    source /opt/ros/jazzy/setup.bash 
    source /opt/turtlebot3_ws/install/setup.bash 
    rm -rf build/ install/ log/
    colcon build --symlink-install
    source install/setup.bash 
    export TURTLEBOT3_MODEL=burger 
    ```

5. Launch custom ROS2 nodes  
   Run the following:
   ```
   ros2 launch my_tb3_world master.launch.py
   ```

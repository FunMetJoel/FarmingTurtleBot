if [ "$(basename "$PWD")" != "ws" ]; then
    echo "Error: You are not in the 'ws' directory."
    echo "Current folder: $(basename "$PWD")"
    echo "Make sure to first setup or join the container and go to /ws, "
    echo "or ask Joël for help"
else
    echo "Confirmed: You are in the 'ws' folder. Rebuilding"
    rm -rf build/ install/ log/
    colcon build --symlink-install
    source install/setup.bash
fi


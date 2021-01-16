# ROSBOT

My adventure of ROS starts here.

# "Requirements"

## Hardware
- Raspberry PI 4 8GB

## Software
- Terminator
    - Shortcuts:
        - Ctrl + Shift + O - Split Window Horizontally
        - Ctrl + Shift + E - Split Window Vertically
- Ubuntu 20.04
- Python3
- ROS2
    - When trying to interact with ros, you need to setup your `~./bashrc` with the following line.
    `source /opt/ros/foxy/setup.bash` this is to avoid resourcing all the time.
- python3-colcon-common-extensions - with autocomplete enabled;
    - ```cd /usr/share/colcon_argcomplete/hook``` and add `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash` to `~/.bashrc`.
- make your ros workspace and source it.
    - make the workspace
    ```
    cd ~
    mkdir ros2_ws
    cd ros2_ws
    mkdir src
    colcon build
    ```
    - `source ~/ros2_ws/install/setup.bash` inside `~/.bashrc`

# Commands
- `colcon build` - to build all packages in the workspace.
    `colcon build --packages-select my_py_pkg` build only selected packages
- `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy` create new **python** ros package for 
- `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp` create new **C++** ros package
- Start node `ros2 run my_py_pkg py_node`
- in case you want to run a file from the `install` directory, make sure you have permissions to run it, but running `chmod +x NODE_NAME_HERE`

# Things to remember

## General
- after building your package, before you can run it, remember to `source ~/.bashrc`.

## Python specific
- When creating your package, remember to include your node inside `entry_node` and `console_scripts`. eg:
    ```python
        # ...
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                "py_node = my_py_pkg.my_first_node:main"
            ],
        },
    ```
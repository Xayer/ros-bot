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
- `colcon build --packages-select <package> --symlink-install` build only selected packages. `--symlink-install` will make a symlink to the package directly, instead of the compiled executeable.
- `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy` create new **python** ros package for 
- `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp` create new **C++** ros package
- Start node `ros2 run my_py_pkg py_node`
- in case you want to run a file from the `install` directory, make sure you have permissions to run it, but running `chmod +x NODE_NAME_HERE`
- Start **ROS2-Web-Bridge** `cd ~/ros2_ws/ros2-web-bridge && npm run wsserver`.
- `ros2 node list` - list a curently running nodes
- `ros2 node info <node>` - list information regarding the selected node.
- `ros2 run <package> <node> --ros-args -r __node:=<new name>` Run a node with a different name. This is useful because all nodes must have unique names.
- `RQT` - will launch a ROS GUI with a collection of plugins.
    - `Plugins > Introspection > Node Graph` draggable UI to see all nodes.
    - Change `Nodes only` to `Nodes/Topic (all)` and uncheck dead sinks, to see all topics running too.
- `ros2 interface show example_interfaces/msg/String` - view what is inside an interface.

# Things to remember

## General
- after building your package, before you can run it, remember to `source ~/.bashrc`.
- in order for `--symlink-install` to work, the file needs to be an executable, so run `chmod +x NODE_NAME_HERE` to make sure it is. 
- you might want to make an alias to this command, as you will use it alot:
    `colcon build --packages-select {package_name} && source ~/.bashrc && ros2 run {package_name} {node_name}`

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
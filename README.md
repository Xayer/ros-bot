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
- `colcon build` - to build the workspace.
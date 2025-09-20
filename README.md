# turtlesim_extensions

The repository contains two example ROS 2 packages:
- turtlesim_ext
- turtlesim_ext_interfaces

The aim of the repository is to show basic ROS functionalities using a turtlesim as an example.

## Building
Clone the repository inside your workspace `src` folder:
```bash
git clone https://github.com/BartlomiejKulecki/turtlesim_extensions.git
```
Go to workspace directory and build:
```bash
colcon build --symlink-install
```

## Running
To run the code use commands:
```bash
ros2 run turtlesim_ext pose_tracker
```
and
```bash
ros2 run turtlesim_ext turtle_bahavior
```

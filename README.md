# Auto Nav

Version: 0.6.0

Code name: N/A

## Description

Controls autonomous navigation and manages the various sub-systems that are required for autonomous navigation. Auto Nav is responsible for communicating with the Control System and sending that information where needed.

## Dependencies

## Build and Run

Ensure that dependencies are either installed and sourced or are in the workspace.

```bash
colcon build --symlink-install
source install/local_setup.bash

ros2 launch launch/launch.py
```

## Quick Example

```bash
# Assuming the workspace is already built and sourced

# Run the launch file
ros2 launch launch/launch.py static_dir:=/home/damon/robotics/AutoNav/static/ site_name:=urc

# Open a new terminal and source the workspace
ros2 action send_goal /make_plan auto_nav_interfaces/action/MakePlan '{current_location: {latitude: 38.415844,longitude: -110.790060},target: {location: {latitude: 38.416508,longitude: -110.800384},type: 0}}'
```

# Auto Nav

Version: 0.2.0

Code name: N/A

## Description

Controls autonomous navigation and manages the various sub-systems that are required for autonomous navigation. Auto Nav is responsible for communicating with the Control System and sending that information where needed.

## Dependencies

-   core_interfaces v0.3.0

## Build and Run

Ensure that dependencies are either installed and sourced or are in the workspace.

```bash
colcon build --symlink-install
source install/local_setup.bash

ros2 launch launch/launch.py
```

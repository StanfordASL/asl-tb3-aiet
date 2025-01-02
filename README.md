# TPS AI-ET Autonomy Stack

This is cloned implementation of the AA174/274 Autonomy Stack

## Setup machine and build instructions.

1. Follow this [guide](https://dafaistudio.notion.site/Installation-Guide-16e777dc07818011ae82f3ed1f07aa99?pvs=4).

## Launch Files
Before launching anything, make sure either a hardware or simulated
turtlebot is running, i.e. run one of the following launches first:
```sh
ros2 launch asl_tb3_driver bringup.launch.py    # on hardware
# or
ros2 launch asl_tb3_sim <world>.launch.py       # on desktop
```
where `<world>` can be chosen from `[root, arena, project_city, maze, signs]`

#### Heading Controller
```sh
ros2 launch asl_tb3_autonomy heading_control.launch.py
```
This launch file achieves pose stabilization control. Target pose can be set from RVIZ. \
**Note**: this controller controls orientation only.

#### Navigator
```sh
ros2 launch asl_tb3_autonomy navigator.launch.py
```
This launch file achieves navigation around obstacles using a switching controller. Target pose
can be set from RVIZ. Planned trajectory from A\* will be visualized in green
and the smoothed trajectory will be visualized in blue.

#### Frontier Explorer
```sh
ros2 launch asl_tb3_autonomy frontier_exp.launch.py
```
This launch file achieves autonomous frontier exploration. Set an initial RVIZ goal to start
the exploration process. When the exploration is done, you should see the following printout
in console:
```sh
Finished exploring
```

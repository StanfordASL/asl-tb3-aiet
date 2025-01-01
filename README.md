<<<<<<< HEAD
# asl-tb3-aiet
=======
# AA174A/274A Reference Autonomy Stack

This is the TA implementation of what a student (group) needs to build throughout the quarter

## Build Instruction

1. Build a turtlebot3 workspace following
   [this guide](https://github.com/StanfordASL/asl-tb3-utils).
2. Clone this repo into a seperate workspace (e.g. `~/my_ws/src`)
    ```sh
    mkdir -p ~/my_ws/src && cd ~/my_ws/src
    git clone https://github.com/StanfordASL/asl-tb3-autonomy.git
    ```
3. Instal dependencies (might not be necessary but just to be safe)
    ```sh
    rosdep update && rosdep install --from-paths ~/my_ws/src -r -i -y
    ```
4. Build and install
    ```sh
    cd ~/my_ws && colcon build --symlink-install
    ```

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
>>>>>>> 51df75c (Initial Commit)

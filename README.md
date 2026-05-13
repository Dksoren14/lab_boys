### Setup:
1. Clone from github:
```
git clone git@github.com:Dksoren14/lab_boys.git
```

2. Setup workspace:
```
cd lab_boys
colcon build
```
Remember that master is only for code we are completely sure works.
When working on code, always push to development first, then when we are sure this works as intendet we merge with master.

3. Check for development branch:
```
git branch -a
```
If the output is:
```
* master
remotes/origin/master
remotes/origin/development
remotes/origin/master
```
Move on to step 4, else contact nearest adult.

4. Switch to development branch:
```
git switch development
```


### To make the robot move:
Testing without the use of the control system
```
ros2 topic pub /model/r100/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Manual move command:
```
ros2 action send_goal /control/chassis/base_command interfaces/action/BaseCommand "{command: 'manual', target_pose: [1.0, 1.0, 0.0]}"
```

Automated move command:
```
ros2 action send_goal /control/chassis/base_command interfaces/action/BaseCommand "{command: 'goto', target_pose: [1.0, 1.0, 0.0]}"
```

Stop command:
```
ros2 action send_goal /control/chassis/base_command interfaces/action/BaseCommand "{command: 'stop', target_pose: []}"
```

Launch simulated vision algorithm:
```
ros2 run sensors sim_aruco_node
```

Launch real vision algorithm:
```
ros2 run sensors r100_aruco_node
```




### Launch Gazebo Simulation:
This opens Gazebo with a simulated environment of the lab, with the R100(Ridgeback)

1. Build workspace (If the code has changed):
```
colcon build
```

2. Source ros:
```
source install/setup.bash
```

3. Use the launch script, to launch gazebo and rviz2:
```
ros2 launch claus claus_sim.launch.py rviz:=true gui:=true control:=true
```
4. Manually moving the robot

In another terminal write, then tap on the original terminal to make the robot move using WASD
```
ros2 action send_goal /control/chassis/base_command interfaces/action/BaseCommand "{command: 'manual', target_pose: [1.0, 1.0, 0.0]}"
```
5. Save the map

In a new terminal. {mapname} is just a placeholder for the name of the file. This should just be changed when saving the map to "laboratory" fx.
```
ros2 run nav2_map_server map_saver_cli -f ~/lab_boys/src/gazebo/maps/{mapname}
```

### NOTE TIL SIG SELV (Søren)/Setup script:
```
cd setup
chmod +x setup.sh
./setup.sh
```

tilføj sudo apt update

sudo apt install ros-${ROS_DISTRO}-unique-identifier-msgs

pip3 install --user pyrealsense2 --break-system-packages :: Den skal hjælpe med cam uden at skal lave virtuel environment


TO the real robot:

IP on dksoren pc: 192.168.120.123 (static dont change)

IP on bjarke pc: 198.168.120.146 (static dont change)

IP for robot: 192.168.120.143 (static dont change)  

Steps to turn on robot:

press turn on button on robot

Press red button on remote, so it blinks green

when the light flashes yellow, press reset button

then take out charger cable

how to turn off million of topics
inside the robot shh
systemctl stop cpr-indoornav.service

source ~/ros-jazzy-ros1-bridge/install/local_setup.bash

ROS_MASTER_URI='http://192.168.120.143:11311' ros2 run ros1_bridge dynamic_bridge --bridge-all-topics



lanch communication with UR5e:
```
roslaunch ridgeback_ur_bringup ridgeback_ur_bringup.launch use_tool_communicatio:=true

```
nano enable_mobile_realsense.sh check if pointcloud is false

cd catkin_ws/src run the script

/front/realsense/color/image_raw

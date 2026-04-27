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
ros2 launch claus_gazebo claus_sim.launch.py rviz:=true gui:=true lab_base:=true
```
4. Manually moving the robot
In another terminal write, then tap on the original terminal to make the robot move using WASD
```
ros2 action send_goal /lab_base/chassis/base_command interfaces/action/BaseCommand "{command: 'manual', target_pose: [1.0, 1.0, 0.0]}"
```
5. Save the map
In a new terminal. {mapname} is just a placeholder for the name of the file. This should just be changed when saving the map to "laboratory" fx.
```
ros2 run nav2_map_server map_saver_cli -f ~/lab_boys/src/claus_sim/claus_gazebo/maps/{mapname}
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

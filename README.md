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
When working 

3. Check for development branch:
```
git branch -a
```
If the output is:
```
* main
remotes/origin/main
remotes/origin/development
```
Move on to step 4, else contact nearest adult.

4. Switch to development branch:
```
git switch development
```


To make the robot move:

ros2 topic pub /model/r100/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.0}}"

ros2 action send_goal /lab_base/chassis/base_command interfaces/action/BaseCommand "{command: 'goto', target_pose: [1.0, 1.0, 0.0]}"

```
cd setup
chmod +x setup.sh
./setup.sh
```

tilføj sudo apt update
sudo apt install ros-${ROS_DISTRO}-unique-identifier-msgs

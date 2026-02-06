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



### Ros usage
So far we only have the minimum for ros to work, so we can run a "hello world" script for the package "lab_manipulator".

1. Build workspace (need to do this after every code change):
```
colcon build
```

2. Source ros
```
source install/setup.bash
```

3. Run lab_manipulator main code:
```
ros2 run lab_manipulator lab_arm
```



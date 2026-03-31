

#--- Install dependencies
if dpkg -s gz-harmonic >/dev/null 2>&1; then
    echo "gz-harmonic already installed"
else
    echo "Installing gz-harmonic..."
    sudo apt install -y gz-harmonic
fi

if dpkg -s ros-jazzy-ros-gz >/dev/null 2>&1; then
    echo "ros-jazzy-ros-gz already installed"
else
    echo "Installing ros-jazzy-ros-gz..."
    sudo apt install -y ros-jazzy-ros-gz
fi

if dpkg -s ros-jazzy-xacro >/dev/null 2>&1; then
    echo "ros-jazzy-xacro already installed"
else
    echo "Installing ros-jazzy-xacro..."
    sudo apt install -y ros-jazzy-xacro
fi

if dpkg -s ros-$ROS_DISTRO-navigation2 >/dev/null 2>&1; then
    echo "ros-$ROS_DISTRO-navigation2 already installed"
else
    echo "Installing ros-$ROS_DISTRO-navigation2..."
    sudo apt install -y ros-$ROS_DISTRO-navigation2
    sudo apt install -y ros-$ROS_DISTRO-nav2-bringup
fi


echo "🏗️ Building workspace..."

cd ~/lab_boys
colcon build 

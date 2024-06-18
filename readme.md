dependencies to install after creating image
-gazebo fortress :

```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt install python3-pip
sudo apt-get install ignition-fortress
sudo apt-get install ros-humble-ros-ign-bridge
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-turtlebot3-gazebo

export TURTLEBOT3_MODEL=waffle/waffle_pi/burger
sudo apt-get install ros-humble-realsense2-camera
sudo apt-get install ros-humble-xacro

## 2. Camera Calibration
sudo apt install ros-humble-camera-calibration-parsers
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-launch-testing-ament-cmake
sudo apt install ros-humble-camera-calibration

sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-sensor-msgs
sudo apt-get install ros-humble-sensor-msgs-py

sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup

git clone https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release

<!-- pip3 install Cython pcl -->

git clone https://github.com/pixmoving-moveit/multi_lidar_calibration_ros2.git


```
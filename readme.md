dependencies to install after creating image
-gazebo fortress :

```
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
sudo apt-get install ros-humble-ros-ign-bridge
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-turtlebot3-gazebo

export TURTLEBOT3_MODEL=waffle/waffle_pi/burger
sudo apt-get install ros-humble-realsense2-camera
sudo apt-get install ros-humble-xacro

```
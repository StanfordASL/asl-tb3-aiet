#!/usr/bin/env bash

echo "Installing apt dependencies..."
sudo apt -qq update && sudo apt upgrade -y
sudo apt install git curl software-properties-common lsb-release wget gnupg git \
                 python3-dev python3-venv python3-pip cmake build-essential vim tmux htop gh -y

echo "Setting up config and vim"
mkdir -p ~/.colcon
git clone https://github.com/alvinsunyixiao/configs.git $HOME/configs
ln -s $HOME/configs/vim/.vimrc ~
ln -s $HOME/configs/tmux/.tmux.conf ~
ln -s $HOME/configs/colcon/defaults.yaml ~/.colcon
curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
vim -E +'PlugInstall --sync' +qa

echo "Installing ROS2 Humble..."
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt -qq update && sudo apt -qq upgrade -y
sudo apt install ros-humble-ros-base ros-dev-tools -y
sudo rosdep init

echo "Installing Gazebo Garden..."
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt -qq update
sudo apt install gz-garden -y

echo "Setting up ROS2 workspace..."
mkdir -p ~/tb_ws/src && cd ~/tb_ws/src
git clone -b humble https://github.com/gazebosim/ros_gz.git
git clone https://github.com/StanfordASL/asl-tb3-driver.git
git clone https://github.com/StanfordASL/asl-tb3-utils.git

echo "Installing ROS2 dependencies..."
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths ~/tb_ws/src -r -i -y

echo "Building ROS2 workspace..."
cd ~/tb_ws
GZ_VERSION=garden colcon build --symlink-install

echo "Installing Python Packages..."
sudo curl -sSL https://raw.githubusercontent.com/StanfordASL/asl-tb3-aiet/refs/heads/main/setup/requirements.txt -o ~/Downloads/requirements.txt
sudo pip3 install -r ~/Downloads/requirements.txt
rm -f ~/Downloads/requirements.txt

echo "Updating ~/.bashrc to include setup scripts"
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source \$HOME/tb_ws/install/local_setup.bash" >> ~/.bashrc
echo "alias update_tb_ws=\$HOME/tb_ws/src/asl-tb3-utils/scripts/update.sh" >> ~/.bashrc
echo "export DISABLE_GUI=0" >> ~/.bashrc
echo "export INSIDE_VM=0" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

echo "alias start_recording='ros2 topic pub /data_collection_command std_msgs/msg/String \"data: 'start'\" -1'" >> ~/.bashrc
echo "alias stop_recording='ros2 topic pub /data_collection_command std_msgs/msg/String \"data: 'stop'\" -1'" >> ~/.bashrc
echo "alias teleop='ros2 run asl_tb3_aiet keyboard_teleop.py'" >> ~/.bashrc

echo "Done!"

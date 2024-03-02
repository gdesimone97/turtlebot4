#!/bin/bash

sudo apt update

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo apt update
sudo apt upgrade -y

sudo apt install -y ros-humble-desktop

echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc

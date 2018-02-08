# Installing ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

source /opt/ros/kinetic/setup.bash

echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Setting up workspace
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# Installing ArduPilot
pip install --upgrade pip
cd ~
git clone git://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip install future pymavlink MAVProxy

echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc

. ~/.bashrc

# Install mavros
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

chmod +x install_geographiclib_datasets.sh

./install_geographiclib_datasets.sh

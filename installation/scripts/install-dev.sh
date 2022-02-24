# updating
sudo apt update
sudo apt upgrade -y

# install ROS prerequisites
sudo apt update
sudo apt install -y curl gnupg2 lsb-release

# install ROS
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-foxy-ros-base
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# install colcon
sudo apt install python3-colcon-common-extensions python3-rosdep2
sudo apt install python-is-python3

# make sure git is installed
sudo apt install -y git

# install ros2_ui prerequisites
sudo apt install -y python3-pip
pip install flask flask-debugtoolbar flask-socketio simple-websocket

## raspi
sudo apt-get install python-rpi.gpio python3-rpi.gpio
sudo apt install python3-gpiozero

# install srosc (rolling dev)
git clone https://github.com/haegelix/ros2_ui.git
cd ros2_ui || exit
echo install it now please
#make build
#sudo make install
# done


# uninstall ROS
# sudo apt remove ~nros-foxy-* && sudo apt autoremove
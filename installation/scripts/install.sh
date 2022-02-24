# updating
echo "Updating your system..."
sudo apt update
sudo apt upgrade -y

# install ROS prerequisites
echo "Installing ROS2 prerequisites (if needed)"
sudo apt install -y curl gnupg2 lsb-release

# install ROS2
echo "Installing RO2"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y ros-foxy-ros-base
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# install colcon & rosdep2
echo "Installing packaged needed to build ROS2-packages"
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep2

# install ros2_ui prerequisites
echo "Installing ros2_ui prerequisites"
sudo apt install -y python3-pip python-is-python3
pip install flask flask-socketio simple-websocket wget
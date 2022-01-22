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

# make sure git is installed
sudo apt install -y git

# install srosc (rolling dev)
git clone https://github.com/haegelix/Simple_ROS_Configurator.git
cd Simple_ROS_Configurator || exit
make build
sudo make install
echo "now install the srosc package using 'sudo dpkg --install <name>.deb'"
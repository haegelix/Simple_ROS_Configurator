# ros2_ui - A UI for ROS2
## Installing
**ros2_ui only got tested on Ubuntu 20.04 running on a RaspberryPi 4. 
No claims can be made that it will run on other systems too.**

### Prerequesites
You can directly run the setup script by
```
bash <(wget -qO- https://raw.githubusercontent.com/haegelix/ros2_ui/main/installation/scripts/install.sh)
```
Note: It's bad practice to run code without checking it before. So please check it out.

Or you could just go visit the file and run the code line by line. [Follow this link](installation/scripts/install.sh)

#### For usage on RaspberryPi
In order to use the builtin nodes for led & button you have to install more packages:
````
sudo apt-get install python3-rpi.gpio
sudo apt-get install python3-gpiozero
````
You can also install those by using pip.

### Installing ros2_ui (release-versions)
Run this to install the current main version:
```
pip install git+https://github.com/haegelix/ros2_ui.git
```

Or use this to install a specific version (here 0.1.0):
```
pip install git+https://github.com/haegelix/ros2_ui.git@0.1.0
```

### Installing ros2_ui (development version)
Please follow these steps:
1. Clone the repo to your local filesystem
2. Switch to branch "develop"
3. Install by using `pip install .`
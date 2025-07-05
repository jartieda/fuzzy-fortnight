# Artisaur Robot

## Overview
Artisaur is a ROS1-based project designed to control the NanoSaur robot. This project includes functionalities for motor control, visual feedback, and robot behavior management.

## Installation
To install the Artisaur package, follow these steps:

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd artisaur_ws
   ```

2. **Build the package**:
   Make sure you have ROS1 installed and sourced. Then run:
   ```bash
   catkin_make
   ```

3. **Source the setup file**:
   After building, source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Usage
To launch the NanoSaur node, use the following command:
```bash
roslaunch artisaur artisaur.launch
```

## Dependencies
Ensure that the following ROS packages are installed:
- `roscpp`
- `rospy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.


## ros1 install on jetson

sudo apt update
sudo apt install -y curl git build-essential libz-dev libbz2-dev libncurses-dev libssl-dev libffi-dev libreadline-dev liblzma-dev libsqlite3-dev
curl https://pyenv.run | bash

pyenv install 3.10.18
pyenv global 3.10.18

sudo apt-get update && apt-get install -y     git     cmake     build-essential     libboost-thread-dev     libboost-system-dev     libboost-filesystem-dev     libboost-regex-dev     libboost-program-options-dev     libconsole-bridge-dev     libpoco-dev     libtinyxml2-dev     liblz4-dev     libbz2-dev     uuid-dev     liblog4cxx-dev     libgpgme-dev     libgtest-dev     python3-pip     tmux     python3     python3-pip     python3-setuptools     python3-empy     python3-nose     python3-pycryptodome     python3-defusedxml     python3-mock     python3-netifaces     python3-gnupg     python3-numpy     python3-psutil     libboost-python-dev     libopencv-dev     libyaml-cpp-dev     libspnav-dev     sqlite3     libsqlite3-mod-spatialite

sudo apt-get install libavcodec-dev 
sudo apt-get install libswscale-dev 

pip install     matplotlib     diffusers     efficientnet_pytorch     einops     vit_pytorch     wandb     prettytable defusedxml netifaces

mkdir -p catkin_ws/src &&     git clone https://github.com/ros-infrastructure/catkin_pkg.git -b 0.5.2 --depth 1 &&     git clone https://github.com/ros-infrastructure/rospkg.git -b 1.5.0 --depth 1 &&     cd catkin_ws/src &&     git clone https://github.com/ros/actionlib.git -b 1.14.0 --depth 1 &&     git clone https://github.com/ros/bond_core.git -b 1.8.6 --depth 1 &&     git clone https://github.com/ros/catkin.git -b 0.8.10 --depth 1 &&     git clone https://github.com/ros/class_loader.git -b 0.5.0 --depth 1 &&     git clone https://github.com/ros/cmake_modules.git -b 0.5.0 --depth 1 &&     git clone https://github.com/ros/common_msgs.git -b 1.13.1 --depth 1 &&     git clone https://github.com/ros/dynamic_reconfigure.git -b 1.7.3 --depth 1 &&     git clone https://github.com/ros/gencpp.git -b 0.7.0 --depth 1 &&     git clone https://github.com/jsk-ros-pkg/geneus.git -b 3.0.0 --depth 1 &&     git clone https://github.com/ros/genlisp.git -b 0.4.18 --depth 1 &&     git clone https://github.com/ros/genmsg.git -b 0.6.0 --depth 1 &&     git clone https://github.com/RethinkRobotics-opensource/gennodejs.git -b 2.0.1 --depth 1 &&     git clone https://github.com/ros/genpy.git -b 0.6.16 --depth 1 &&     git clone https://github.com/ros/message_generation.git -b 0.4.1 --depth 1 &&     git clone https://github.com/ros/message_runtime.git -b 0.4.13 --depth 1 &&     git clone https://github.com/ros/nodelet_core.git -b 1.10.2 --depth 1 &&     git clone https://github.com/ros/pluginlib.git -b 1.13.0 --depth 1 &&     git clone https://github.com/ros/ros.git -b 1.15.8 --depth 1 &&     git clone https://github.com/ros/ros_comm.git -b 1.16.0 --depth 1 &&     git clone https://github.com/ros/ros_comm_msgs.git -b 1.11.3 --depth 1 &&     git clone https://github.com/ros/ros_environment.git -b 1.3.2 --depth 1 &&     git clone https://github.com/ros/rosbag_migration_rule.git -b 1.0.1 --depth 1 &&     git clone https://github.com/ros/rosconsole.git -b 1.14.3 --depth 1 &&     git clone https://github.com/ros/rosconsole_bridge.git -b 0.5.4 --depth 1 &&     git clone https://github.com/ros/roscpp_core.git -b 0.7.2 --depth 1 &&     git clone https://github.com/ros/roslisp.git -b 1.9.25 --depth 1 &&     git clone https://github.com/ros/rospack.git -b 2.6.2 --depth 1 &&     git clone https://github.com/ros/std_msgs.git -b 0.5.13 --depth 1 &&     git clone https://github.com/ros-drivers/usb_cam.git -b 0.3.7 --depth 1 &&     git clone https://github.com/ros-drivers/joystick_drivers.git -b 1.15.1 --depth 1 &&     git clone https://github.com/ros/diagnostics.git -b 1.11.0 --depth 1 &&     git clone https://github.com/ros-perception/vision_opencv.git -b 1.16.2 --depth 1 &&     git clone https://github.com/ros-perception/image_common.git -b 1.12.0 --depth 1 &&     git clone https://github.com/ros/roslint.git -b 0.12.0 --depth 1 

cd catkin_pkg && python3 setup.py install &&     cd ../rospkg && python3 setup.py install

catkin_ws/

export PATH=~/.pyenv/versions/3.10.18/bin/:$PATH


pip install empy==3.3.4

./src/catkin/bin/catkin_make install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF

source /home/jartieda/catkin_ws/install/setup.bash


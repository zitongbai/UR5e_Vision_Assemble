# UR5_Vision_Assemble
Use ROS2 and Gazebo to simulate 2 UR5 assembling objects. 

> ⚠️ **Notice:** This repository is no longer actively maintained.  
> While you are welcome to explore and use the code, please note that updates and issue responses may be infrequent.  
> Thank you for your interest and understanding.

# Environment

* Ubuntu 22.04
* ROS2 humble

# Installation

Before installation, you need some dependencies installed.

## Gazebo classic

Unlike ROS, Gazebo is not installed when you install ROS2. You need to install it manually.

```bash
sudo apt update
sudo apt install gazebo
```

## Moveit2

```bash
sudo apt install ros-humble-moveit*
```

## ros2 control
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
## dependencies of image

```bash
sudo apt install ros-humble-image-pipeline
sudo apt install ros-humble-compressed-image-transport
sudo apt install ros-humble-compressed-depth-image-transport
sudo apt install ros-humble-vision-msgs
```

## Clone this repo

Now clone this repo
```bash
cd ~
git clone git@github.com:zitongbai/UR5e_Vision_Assemble.git --recurse-submodules
```

## setup environment for yolov5

You can refer to the [README.md](https://github.com/zitongbai/UR5e_Vision_Assemble/blob/main/src/vision/README.md) in the vision package. 

You should not only set up the conda environment, but also train the yolov5 and move the weight files for later usage. 

Under the original python environment, you should upgrade the numpy version
```bash
pip install -U numpy
```

## Build

Build the repo
```bash
cd ~/UR5e_Vision_Assemble
colcon build --symlink-install
```

# Usage

Start the simulation environment
```bash
cd ~/UR5e_Vision_Assemble
source install/setup.bash
ros2 launch bringup simulation.launch.py
```

Launch the demo
```bash
# in another terminal
cd ~/UR5e_Vision_Assemble
source install/setup.bash
ros2 launch ur5e_gripper_control demo4.launch.py
```


# Some notes

## Before usage

make sure you have all the dependences installed

make sure you have local gazebo model in `~/.gazebo/models`

## 记录一下踩过的各种坑

1. 调用moveit控制的那个launch文件也需要单独加载kinematics等参数（robot_description之类的倒是不用，好奇怪）
2. xml文件的注释中不要含有冒号:
3. 记得区分${xxx}和$(arg xxx)
4. 仿真需要设置`use_sim_time=true`，而且目前来看最好在launch文件里面来设置
5. 因为固定link的tf没有被发布出来导致深度相机返回的数据有问题

# Acknowledgment

* Gazebo grasp plugin for ROS2 [gazebo_pkgs](https://github.com/kongoncharuk/gazebo-pkgs)
* [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
* [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
* [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

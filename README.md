# UR5_Vision_Assemble
Use ROS2 and Gazebo to simulate 2 UR5 assembling objects. 

## Environment

* Ubuntu 22.04
* ROS2 humble
* Gazebo classic



# 记录一下踩过的各种坑

1. 调用moveit控制的那个launch文件也需要单独加载kinematics等参数（robot_description之类的倒是不用，好奇怪）
2. xml文件的注释中不要含有冒号:
3. 记得区分${xxx}和$(arg xxx)
4. 仿真需要设置`use_sim_time=true`，而且目前来看最好在launch文件里面来设置
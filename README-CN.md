# <center>CR5Robot</center>

# 1. 源码编译
## ubuntu16.04
### 下载源码
```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/CR5_ROS.git -b kinetic-devel

cd $HOME/catkin_ws
```
### 编译
```
catkin_make
```
### 设置环境变量
```
source $HOME/catkin_ws/devel/setup.bash
```

## ubuntu18.04
###下载源码
```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/CR5_ROS.git -b melodic-devel

cd $HOME/catkin_ws
```
### 编译
```
catkin_make
```
### 设置环境变量
```
source $HOME/catkin_ws/devel/setup.bash
```

# 2. 设置机器人类型
### 若为 CR3 机械臂，则使用如下命令设置机械臂类型
```
echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
source ~/.bashrc
```
### 若为 CR5 机械臂，则使用如下命令设置机械臂类型
```
echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
source ~/.bashrc
```
### 若为 CR10 机械臂，则使用如下命令设置机械臂类型
```
echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
source ~/.bashrc
```
### 若为 CR16 机械臂，则使用如下命令设置机械臂类型
```
echo "export DOBOT_TYPE=cr16" >> ~/.bashrc
source ~/.bashrc
```
# 3. 示例演示

## 在仿真环境下使用

1. ## rviz 显示

    ```
    roslaunch dobot_description display.launch
    ```

    可通过 joint_state_publisher_gui 调节各关节的角度，在 rviz 上看到其显示效果

    ![rviz显示](./rviz.jpg)


2. ## moveit 控制
    * 使用如下命令启动 moveit
    ```
    roslaunch dobot_moveit demo.launch
    ```
    * 鼠标将关节拖到任意的角度，点击 "Plan and Execute" 即可看到运行效果

    ![moveit显示](./moveit.gif)

3. ## gazebo 仿真
    * 使用如下命令启动 gazebo
    ```
    roslaunch dobot_gazebo gazebo.launch 
    ```
    * 同样，您可以使用MoveIt!控制gazebo里的仿真机器人
    * 设置MoveIt!允许运动规划运行的节点,dobot类型需要对应 
    ```
    roslaunch cr5_moveit cr5_moveit_planning_execution.launch  sim:=True
    ```
    * 启动RViz的配置包括MoveIt!运动规划插件运行,dobot类型需要对应 
    ```
    roslaunch cr5_moveit moveit_rviz.launch config:=True
    ```
    * 鼠标将关节拖到任意的角度，点击 "Plan and Execute" 即可看到运行效果
## 4. 控制真实机械臂

* **使用如下命令连接机械臂, robot_ip 为实际机械臂所对应的IP地址**
    ```
    roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1
    ```

* **使用如下命令启动 Moveit**
    ```
    roslaunch dobot_moveit moveit.launch
    ```

* **在 rviz 中添加 DobotControl 插件控制面板，用来 使能机械臂**
    1. 点击 rviz 工具栏上的 Panels --> "Add New Panel"
    2. 选中 DobotControl, 再点击 “OK”
    3. 点击 “EnableRobot” 使机械臂
    4. 当状态样上显示 “Enable” “Connected” 表示机械臂已连接和使能，即可通过 Moveit 控制机械臂

    ![DobotControl](./cr5control.jpg)


# 自定义功能开发

    dobot_bringup 中定义了 msg 和 srv，用户可通过这些底层 msg 和 srv 实现对机械臂的控制

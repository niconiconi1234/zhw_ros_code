# 智慧湾项目 机械臂部分
由于JAKA机械臂的ROS驱动只有ROS1版的，所以我们使用ROS1版的驱动，然后通过ROS2的ros1_bridge来实现ROS1和ROS2的通信。
## 环境
- Ubuntu 20.04
- ROS1 noetic
- ROS2 foxy
## build方法
1. 在ROS1的工作空间下编译JAKA的ROS1驱动。启动一个干净的终端，输入以下命令：
```bash
```bash
source /opt/ros/noetic/setup.bash
cd ./ros1_ws
catkin_make
```
2. 在ROS2的工作空间下编译除ros1_bridge以外的所有包。启动一个干净的终端，输入以下命令：
```bash
source /opt/ros/foxy/setup.bash
cd ./ros2_ws
colcon build --symlink-install --packages-skip ros1_bridge
```
3. 在ROS2的工作空间下编译ros1_bridge。首先删除`ros2_ws/src/ros1_bridge/COLCON_IGNORE`文件，然后分别source ROS1和ROS2的环境，然后编译ros1_bridge。启动一个干净的终端，输入以下命令：
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/local_setup.bash
source ./ros1_ws/devel/setup.bash
source ./ros2_ws/install/local_setup.bash
cd ./ros2_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

4. 创建`ros2_ws/src/ros1_bridge/COLCON_IGNORE`文件，防止下次编译时编译ros1_bridge。以后ros1_bridge如果需要桥接新的带有自定义消息的ros1话题和服务，则需要删除该文件，然后重新编译ros1_bridge。

5. 最后重新编译一下ROS2工作空间。启动一个干净的终端，输入以下命令
```bash
source /opt/ros/foxy/setup.bash
cd ./ros2_ws
colcon build
```


## 测试ros1_bridge是否生效
1. 启动JAKA ROS1驱动。启动一个终端，输入以下命令：
```bash
source /opt/ros/noetic/setup.bash
source ./ros1_ws/devel/setup.bash
roslaunch jaka_ros_driver start.launch
```
2. 启动ros1_bridge。启动一个终端，输入以下命令：
```bash
source /opt/ros/noetic/setup.bash
source ./ros1_ws/devel/setup.bash
source /opt/ros/foxy/setup.bash
source ./ros2_ws/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics # 强制桥接所有的topic，不管是否有订阅者
```
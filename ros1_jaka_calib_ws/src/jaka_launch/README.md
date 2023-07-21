使用以下代码一键启动
- JAKA机械臂的moveit功能包和rviz可视化工具
- 根据手眼标定得到的参数，将camera_frame和Link_6的关系发布到tf坐标系中
- 启动ar_track_alvar识别二维码(marker)，识别到的ar_marker的位姿发布到tf坐标系中
```bash
roslaunch jaka_launch start.launch
```
![demo](pics/图片%201.png)
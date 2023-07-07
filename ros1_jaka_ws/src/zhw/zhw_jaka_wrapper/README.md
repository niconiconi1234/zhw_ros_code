# zhw_jaka_wrapper
使用前，需要去`jaka_ros_driver/launch/start.launch`，将`robot_ip`修改为JAKA电控柜的ip。
```xml
<!--将192.168.31.7修改成JAKA电控柜的ip-->
<param name="robot_ip" value="192.168.31.7" type="str" />
```
随后使用以下命令启动
```
roslaunch zhw_jaka_wrapper zhw_jaka_wrapper.launch
```
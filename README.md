# ros2 humble moveit

Please add a **.rules file to /etc/udev/rules.d to open the serial port.  
If you want to test the code by publishing topics, you could run the following instructions:  
```
ros2 topic pub -1 /arm_request std_msgs/msg/UInt8 'data: "0"'  
```
or  
```
ros2 run data_process fake_js_pub_node  
```
Besides, you could implementing code self start upon power-up through adding launch.sh to gnome-session-properties.  
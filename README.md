# egh450_robin_servo_control
An example package to publish servo/motor controls to robin. This example works by using the PWM Override interfaces made available by [MAVROS](http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.rc_io) and [robin](https://github.com/qutas/robin/blob/master/documents/INTERFACING.md).

## Dependencies
```sh
sudo apt install ros-kinetic-mavros-msgs
```

## Download & Compile
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/egh450_robin_servo_control
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Running the Example
This example will load in parameters set in the launch file, which allows you to choose:
- The channel to override
- The low/high PWM values to send
- The default position of the servo

The topic `/servo_control/deploy_payload` is subscribed to, allowing you to send an std_msgs/Bool (`true` or `false`) that will trigger the servo to open/close.

The topic `/mavros/rc/override` is published to with the correct override sequence that will set the specific channel to the override value, and leave all other channels as they are (i.e. will only override one channel, and won't turn of the motors). The UAV will need to be armed for this to work.

Run the node with:
```sh
roslaunch egh450_robin_servo_control servo_control.launch
```

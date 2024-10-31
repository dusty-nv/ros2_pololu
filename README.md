# ROS2 Nodes for Pololu Controllers

Pololu drivers for Maestro, Jrk, and Simple Motor Controller

```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/dusty-nv/ros2_polulu
rosdep install --from-paths src -y --ignore-src
colcon build
source install/local_setup.bash
ros2 pkg list | grep pololu
ros2 run pololu maestro
ros2 run pololu smc
```

#### Joystick notes

* PS4 pairing:  hold down `SHARE` button when off, then hold `PS` button until rapidly flashes white

* include `--device /dev/input` in `docker run` commands

* https://github.com/HarvestX/PlayStation-JoyInterface-ROS2
* https://github.com/Ar-Ray-code/ps_ros2_common
* https://github.com/husarion/teleop-twist-joy-docker
* https://doc.iohub.dev/jarvis/Ym9vazovLy9jXzIvc180L0lOVFJPLm1k/Robot_Teleop_using_PS4_controller.md
  * CLI bluetooth pairing with `hcitool scan`
  
```
apt-get install -y jstest-gtk joystick evtest
ros2 run joy joy_enumerate_devices
ros2 run joy joy_node --ros-args --log-level debug
ros2 topic echo /joy
ros2 run teleop_twist_joy teleop_node
ros2 topic echo /cmd_vel
# hold down R1 to enable teleop
```

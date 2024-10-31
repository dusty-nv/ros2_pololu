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
```

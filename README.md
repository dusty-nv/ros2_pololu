# ROS2 Nodes for Pololu Controllers

Pololu drivers for Maestro, Jrk, and Simple Motor Controller.  Testing on ROS2 Humble, Jetson Orin, and JetPack 6.

### Setup Container

> [!NOTE]  
> If you're using JetPack 6 + Ubuntu 22.04, you can install Humble from the [Isaac Apt Repository](https://nvidia-isaac-ros.github.io/getting_started/isaac_apt_repository.html) in your host OS without needing containers.  

For other ROS distros or versions of JetPack/Ubuntu, to enable CUDA support it's recommended to use containers that have built ROS from source.  There are many ROS/ROS2 container images on DockerHub ([`dustynv/ros`](https://hub.docker.com/r/dustynv/ros/tags)) that were built with CUDA enabled.

These containers were built using [jetson-containers](https://github.com/dusty-nv/jetson-containers), which you can install like this to also find and run containers:

```
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```

Also see the [System Setup](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md) from jetson-containers to tweak some system settings for swap memory, power modes, and to store the containers and data on attached high-capacity storage (like NVMe)

### Start Container

Run this to pull or build a ROS2 Humble image, and mount in an external workspace from the host during development:

```
# your user's ~/ros2_ws directory will be mounted in the container under /ros2_ws
# you can make edits from the host, and build it into a Dockerfile for deployment
jetson-containers run -v ~/ros2_ws:/ros2_ws $(autotag ros:humble-desktop)
```
> <sup>[`jetson-containers run`](/docs/run.md) launches [`docker run`](https://docs.docker.com/engine/reference/commandline/run/) with some added defaults (like `--runtime nvidia`, mounted `/data` cache and devices)</sup><br>
> <sup>[`autotag`](/docs/run.md#autotag) finds a container image that's compatible with your version of JetPack/L4T - either locally, pulled from a registry, or by building it.</sup>


The `jetson-containers run` command expands to a `docker run` command that launches the container for your version of JetPack along with some added defaults, like for dynamically detecting and mounting device nodes used for sensors and I/O. 

Depending on what your L4T version is (you can check this under `/etc/nv_tegra_release`), you can specify it manually or provide your own:

```
# this is if you know the specific container image you want to run
jetson-containers run -v ~/ros2_ws:/ros2_ws dustynv/ros:humble-desktop-r36.4.0
```

You can also put these into docker-compose if preferred, or check the `docker run` command it invokes and run that directly.

### Build from Source

Once inside the container, navigate to the mounted workspace and clone/compile this repo:

```
# download the package
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/dusty-nv/ros2_pololu

# install dependencies
rosdep install --from-paths src -y --ignore-src src/ros2_pololu

# compile it
colcon build --packages-select pololu --symlink-install
```

After sourcing the workspace overlay, you should find that ROS can discover the `polulu` package:

```
source install/local_setup.bash
ros2 pkg list | grep pololu
```

### Launching Nodes

You can directly run the nodes like this for example:

```
ros2 run pololu maestro
ros2 run pololu smc
```

Or there are launch files included (found under [`launch/`](launch/))

```
ros2 launch pololu teleop_smc.launch.py
```

This will launch a [joystick node](https://index.ros.org/p/joy/) connected to nodes for Simple Motor Controllers.  By default, you will need to hold down a button on the joystick (for example `R1` on PS3/PS4 controllers) to engage telop mode.

### Joystick (Teleop)

My notes for setting up PS3/PS4 controllers over bluetooth:

* PS4 pairing:  hold down `SHARE` button when off, then hold `PS` button until rapidly flashes white
* Include `--device /dev/input` in `jetson-container run` or `docker run` commands
  * `jetson-containers run -v ~/ros2_ws:/ros2_ws --device /dev/input $(autotag ros:humble-desktop)` 

* https://github.com/HarvestX/PlayStation-JoyInterface-ROS2
* https://github.com/Ar-Ray-code/ps_ros2_common
* https://github.com/husarion/teleop-twist-joy-docker
* https://doc.iohub.dev/jarvis/Ym9vazovLy9jXzIvc180L0lOVFJPLm1k/Robot_Teleop_using_PS4_controller.md
  * CLI bluetooth pairing with `hcitool scan`
  
#### Testing Connectivity

Run this outside of container first, then inside (having mounted `--device /dev/input` when you started the container)

```
# install these joystick utilities
apt-get install -y jstest-gtk joystick evtest

# run this test app to view the state
jstest-gtk
```

These devices typically get mapped to `/dev/input/js0`. After you confirm it to be recieving valid data, try it from ROS next:

```
# this will list the input devices
ros2 run joy joy_enumerate_devices

# manually run the joy_node for testing
ros2 run joy joy_node --ros-args --log-level debug

# or with a more specific device config
ros2 run joy joy_node --ros-args -p device_id:=0

# in another terminal, run this to watch the messages
ros2 topic echo /joy
ros2 run teleop_twist_joy teleop_node
ros2 topic echo /cmd_vel
```

#### Controller Notes

    
* `diff_drive_controller` https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
* `twist_mux` https://robofoundry.medium.com/controlling-a-robot-with-multiple-inputs-using-twist-mux-4535b8ed9559
* `twist` in Nanosaur: https://github.com/rnanosaur/nanosaur_robot/blob/master/nanosaur_base/nanosaur_base/nanosaur.py
* https://robotics.snowcron.com/robotics_ros2/dif_drive_intro.htm
* https://robotics.stackexchange.com/questions/105443/configure-diff-drive-controller-in-open-loop-on-a-real-robot
* https://bunchofcoders.github.io/basic_bocbot/
    

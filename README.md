# minim5robo_bridge
MiniM5Robo ros2 package

## Setup micro_ros
```bash
source /opt/ros/humble/setup.bash

mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep install --from-paths src --ignore-src -y
colcon build

source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

## Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/eieioF11/minim5robo_bridge.git
cd ~/ros2_ws/
colcon build --symlink-install
```

## Launch
```bash
source ~/ros2_ws/install/local_setup.bash
```
### run micro_ros_agent
```bash
ros2 launch minim5robo_bridge micro_ros_agent.launch.py
```

### run minim5robo_bridge
```bash
ros2 launch minim5robo_bridge bringup.launch.py
```

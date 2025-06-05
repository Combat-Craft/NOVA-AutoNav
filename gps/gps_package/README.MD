# 🛰️ Neo-M9N GPS ROS2 Package

This is a ROS2 Python package that reads raw NMEA GPS data from a Neo-M9N GPS module (via Arduino) and publishes it as a `sensor_msgs/NavSatFix` topic.

## 🚀 Quickstart

### 1. Clone and Set Up ROS2 Workspace

```bash
mkdir -p ~/gps_ws/src
cd ~/gps_ws/src
git clone https://github.com/your-username/gps_package.git
cd ..
colcon build --packages-select gps_package
source install/local_setup.bash
```

---

### 2. Upload Arduino GPS Serial Sender

Upload the GPS sender code to your Arduino:
📎 [Arduino NMEA Sender Code](https://github.com/Monti-1/gps.git)

Ensure the Neo-M9N GPS is properly wired and the baud rate matches (`115200` by default).

---

### 3. Run the Node

```bash
ros2 run gps_package gps_node
```

With arguments (e.g. real GPS only):

```bash
ros2 run gps_package gps_node --ros-args -p port:=/dev/ttyACM0 -p simulated_data:=False
```

---

## 🔧 Troubleshooting

- Use `ls /dev/tty*` to find your port
- Add user permissions: `sudo usermod -a -G dialout $USER && sudo chmod 666 /dev/ttyACM0`
- Check if port is in use: `sudo lsof /dev/ttyACM0`
- Kill interfering process: `sudo kill -9 <PID>`
- Reset ROS2 daemon:
```bash
ros2 daemon stop
ros2 daemon start
```

---

## 🧰 Dependencies

```bash
pip install pyserial
sudo apt install ros-humble-rmw-cyclonedds-cpp screen
```

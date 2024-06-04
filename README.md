## ROS wrapper for FLIR Boson data capture
This is a ROS package for capturing and publishing data from FLIR Boson thermal camera models with a USB interface. The codebase is built upon [BosonUSB](https://github.com/FLIR/BosonUSB), a repository owned by FLIR.

## How to use

### Prerequisite
Please ensure that ROS and OpenCV are installed before using this package. This code is verified to operate in an environment with ROS Noetic and OpenCV 4.4.0.

### How to install
```
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
cd src && git clone https://github.com/tag-nav/flir_boson_pub.git && cd ..
catkin build flir_boson_pub
source devel/setup.bash
```

### How to run
```
roslaunch flir_boson_pub boson_640.launch port:=${PORT} dtype:=${DTYPE}
```
Replace `${DTYPE}` and `${PORT}` with appropriate values:
* `PORT`: [string] Port name for the USB device (default: `/dev/video0`).
* `DTYPE`: [int] Data type to stream, either 1 (default: raw data in 16-bit) or 0 (compressed data in 8-bit with automatic gain correction (AGC)).

### To-dos
- [ ] Ensure both data types (16-bit raw data and 8-bit compressed data) are published in the `sensor_msgs::Image` message type with proper data encodings (`moono16` and `mono8`).
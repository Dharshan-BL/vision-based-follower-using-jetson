# Object Detection using Realsense Depth Camera (Humble)

## Setting up Intel-Realsense with Jetson + ROS2 wrapper
The official realsense instructions for jetson might be confusing. This instruction aims to clear it all out.

You essentially need 3 things: ros2-humble, librealsense2 SDK, ros-humble-realsense2 wrapper. Installing librealsense2 SDK using deb pkgs, provides some compatibility issues with the Jetson Kernel (especially depending on the Jetpack verson. In my case, I used a Jetson AGX Orin with Jetpack 6.2 and realsense D435i).

- Install ros-humble pkgs from the official documentation page on the Jetson. Make sure you source it!
  ```
  source /opt/ros/humble/setup.bash
  ```
- Connect your RealSense camera and check if your device recognizes it:
  ```
  lsusb
  ```
- After this, you can disconnect the device and install the librealsense using `libuvc_installation.sh` (refer: [libuvc-installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md))
  ```
  wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
  chmod +x ./libuvc_installation.sh
  ./libuvc_installation.sh
  ```
- It takes a while. But once the above step is completed you must have SDK. Now you can connect your device and check if the SDK recognizes your device using:
  ```
  rs-enumerate-devices
  ```
  NOTE: If you don't see any devices, it must arise from mismatch with your Jetpack version or a pre-installed `librealsense2` (either deb install or source installed)
- Now, we install the [ros-wrapper](https://github.com/IntelRealSense/realsense-ros) through source and build it using `colcon` (Since deb install also installs `librealsense2`)
  ```
  mkdir ~/realsense_ros_ws/src -p
  cd ~/realsense_ros_ws/src
  git clone https://github.com/IntelRealSense/realsense-ros.git
  cd ~/realsense_ros_ws
  colcon build --symlink-install
  ```
- Source the install folder(Optionally you can also add it to your `.bashrc`):
  ```
  source ~/realsense_ros_ws/install/local_setup.bash
  ```
- You can verify the sensor readability using the `rs-enumerate-devices` and also run any example launch file in the environment!
  ```
  ros2 launch realsense2_camera rs_pointcloud_launch.py
  ```

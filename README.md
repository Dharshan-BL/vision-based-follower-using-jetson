# Object Detection using Intel Realsense D435i Depth Camera (Humble)

## Setting up Intel-Realsense with Jetson AGX orin + ROS2 wrapper
The official realsense instructions for jetson might be confusing and you might run into other issues. I would like to provide a clear instructions here.

You essentially need to get the following things right: 
 - The Jetpack version (+ CUDA)
 - The firmware on D435i
 - The SDK installation for `librealsense2`
 - And the `realsense-ros` wrapper.
   
  Installing librealsense2 SDK using deb pkgs, provides some compatibility issues with the Jetson Kernel (especially depending on the Jetpack verson. In my case, I used a Jetson AGX Orin with Jetpack 6.2 and realsense D435i).

### Jetpack version + CUDA
- Ensure that you have the `Jetpack 6.2` version
- (Optional) for CUDA, install it with:
  ```
  sudo apt install cuda-toolkit-12-6
  ```
- You can verify the installation using `nvcc --version`
- Add the cuda root directory:
  ```
  echo "export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.6" >> ~/.bsahrc
  echo "export CUDA_BIN_PATH=/usr/local/cuda-12.6" >> ~/.bsahrc
  ```
### RealSense SDK `librealsense2`
- For this make sure your device is disconnected, I used the [libuvc_installation.sh](https://github.com/IntelRealSense/librealsense/blob/master/doc/libuvc_installation.md). You can simply run it by:
  ```
  wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
  chmod +x ./libuvc_installation.sh
  ./libuvc_installation.sh
  ```  
- This might take a while. Meanwhile, hydrate yourself! 
- Once it is done, you can connect your realsense device now and verify it using
  ```
  rs-enumerate-devices
  ```
  NOTE: If you don't see any devices, it must arise from mismatch with your Jetpack version or a pre-installed `librealsense2` (either deb install or source installed)

### Firmware for D435i
- Get the firmware version `5.13.0.50` from the [intel realsense firmware releases](https://dev.intelrealsense.com/docs/firmware-releases-d400)
- Extract the zip file and navigate to the extracted folder. You must see the file `Signed_Image_UVC_5_13_0_50.bin` (best to verify the file name)
- Connect your RealSense camera and check if your device recognizes it:
  ```
  lsusb
  ```
- Flash the firmware. **DO NOT DISCONNECT THE SENSOR**
  ```
  rs-fw-update -s 725112060411 -f Signed_Image_UVC_5_13_0_50.bin
  ```
- Once it is done, you can verify the firmware by running `rs-fw-update`

### RealSense-ROS2 Wrapper
Since, the deb-packages also installs `librealsense2` SDK by default we have to build this on eby source. 

- Now, we install the [realsense-ros-wrapper](https://github.com/IntelRealSense/realsense-ros) through source and build it using `colcon build`
  ```
  mkdir ~/realsense_ros_ws/src -p
  cd ~/realsense_ros_ws/src
  git clone https://github.com/IntelRealSense/realsense-ros.git
  cd ~/realsense_ros_ws
  ```
- Build it
  ```
  colcon build --symlink-install
  ```
- Source it (Optionally you can also add it to your `.bashrc`):
  ```
  source ~/realsense_ros_ws/install/local_setup.bash
  ```
- You can verify the sensor readability using the `rs-enumerate-devices` and also run any example launch file in the environment!
  ```
  ros2 launch realsense2_camera rs_pointcloud_launch.py
  ```

# RGBD Data Collection
Get RGBD(image and depth) data from "Intel Realsense D435i", "Zed2", "PMD pico monstar" and using Relay(for the night env).

<img src="docs/img/camera_setting.jpg" width="30%" height="30%"></img>

## SDK

### Intel RealSense D435i (Active IR Stereo Camera)

The python wrapper for Intel RealSense SDK 2.0 provides the C++ to Python binding required to access the SDK.

The easiest way to install this library is using pip:
```
pip install pyrealsense2
```

### Zed2 (Stereo Camerea)

To start using the ZED SDK in Python, you should refer to this site:

https://www.stereolabs.com/docs/app-development/python/install/


### PMD pico monstar (ToF Camera)

To start using the Royale SDK, you should refer to this site:

https://pmdtec.com/picofamily/software/

## About USB relay

<img src="docs/relay/usb_relay.jpg" width="20%" height="20%"></img>

please check below README.md about a USB relay information

https://github.com/Jstar0525/RGBD-Data-Collection/tree/master/usb_relay

## Data

dataset is collected in **data** folder

### intel RealSense D435i

* ir_on : IR projector turn on and get gray stereo images and depth
  * left_date_time.png
  * right_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
#### data/realsense/ir_on
<img src="data/realsense/ir_on/left_20200910_142443.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/right_20200910_142443.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/depth_map_20200910_142443.png" width="30%" height="30%"></img>

* rgb : IR projector turn on and get rgb image and depth align with rgb
  * rgb_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
#### data/realsense/rgb
<img src="data/realsense/rgb/rgb_20200910_142443.png" width="30%" height="30%"> <img src="data/realsense/rgb/depth_map_20200910_142443.png" width="30%" height="30%">

* ir_off : IR projector turn off and get gray stereo images and depth
  * left_date_time.png
  * right_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
#### data/realsense/ir_off
<img src="data/realsense/ir_off/left_20200910_142445.png" width="30%" height="30%"></img> <img src="data/realsense/ir_off/right_20200910_142445.png" width="30%" height="30%"></img> <img src="data/realsense/ir_off/depth_map_20200910_142445.png" width="30%" height="30%"></img>

### Zed2

* get rgb stereo images, depth, disparity and confidence
  * left_date_time.png
  * right_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
  * disparity_date_time.npy
  * confidence_date_time.npy

### data/zed
<img src="data/zed/left_20200910_142449.png" width="30%" height="30%"></img> <img src="data/zed/right_20200910_142449.png" width="30%" height="30%"></img> <img src="data/zed/depth_map_20200910_142449.png" width="30%" height="30%"></img>
<img src="data/zed/disparity_map_20200910_142449.png" width="30%" height="30%"></img> <img src="data/zed/confidence_map_20200910_142449.png" width="30%" height="30%"></img>

### PMD pico monstar

* distort : get origimal gray image and depth
  * gray_img_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
#### data/pico/distort
<img src="data/pico/distort/gray_img_20200910_142450.png" width="30%" height="30%"></img> <img src="data/pico/distort/depth_map_20200910_142450.png" width="30%" height="30%"></img>  

* undistort : get distortion correction gray image and depth
  * gray_img_date_time.png
  * depth_date_time.npy
  * depth_map_date_time.png
#### data/pico/undistort
<img src="data/pico/undistort/gray_img_20200910_142450.png" width="30%" height="30%"></img> <img src="data/pico/undistort/depth_map_20200910_142450.png" width="30%" height="30%"></img>

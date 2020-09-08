# RGB-Data-Collection
Get data from "Intel Realsense D435i", "Zed2", "PMD pico monstar" and using Relay.

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

## Data

collect dataset in **data** folder

### intel realsense D435i

collects **realsense** dataset to like this:

* ir_on : IR projector turn on and get gray stereo images and depth
* rgb : IR projector turn on and get rgb image and depth align with rgb
* ir_off : IR projector turn off and get gray stereo images and depth

#### realsense/ir_on
<img src="data/realsense/ir_on/left_20200827_175655.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/right_20200827_175655.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/depth_map_20200827_175655.png" width="30%" height="30%"></img>

#### realsense/rgb
<img src="data/realsense/rgb/rgb_20200827_175655.png" width="30%" height="30%"> <img src="data/realsense/rgb/depth_map_20200827_175655.png" width="30%" height="30%">

#### realsense/ir_off
<img src="data/realsense/ir_off/left_20200827_175657.png" width="30%" height="30%"></img> <img src="data/realsense/ir_off/right_20200827_175657.png" width="30%" height="30%"></img> <img src="data/realsense/ir_off/depth_map_20200827_175657.png" width="30%" height="30%"></img>

### Zed2

collects **Zed2** dataset to like this:

* get rgb stereo images, depth, disparity and confidence

### zed
<img src="data/zed/left_20200827_175701.png" width="30%" height="30%"></img> <img src="data/zed/right_20200827_175701.png" width="30%" height="30%"></img> <img src="data/zed/depth_map_20200827_175701.png" width="30%" height="30%"></img>

### PMD pico monstar

collects **PMD pico monstar** dataset to like this:

* distort : get origimal gray image and depth
* undistort : get distortion correction gray image and depth

#### pico/distort
<img src="data/pico/distort/gray_img_20200827_175703.png" width="30%" height="30%"></img> <img src="data/pico/distort/depth_map_20200827_175703.png" width="30%" height="30%"></img>

#### pico/undistort
<img src="data/pico/undistort/gray_img_20200827_175703.png" width="30%" height="30%"></img> <img src="data/pico/undistort/depth_map_20200827_175703.png" width="30%" height="30%"></img>

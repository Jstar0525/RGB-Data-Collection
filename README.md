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

Data collected in **data** folder

### realsense

Dataset collecte to like this:

* ir_on : IR projector turn on and get gray stereo images and depth
* rgb : IR projector turn on and get rgb image and depth align with rgb
* ir_off : IR projector turn off and get gray stereo images and depth

#### ir_on
<img src="data/realsense/ir_on/left_20200827_175655.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/right_20200827_175655.png" width="30%" height="30%"></img> <img src="data/realsense/ir_on/depth_map_20200827_175655.png" width="30%" height="30%"></img>

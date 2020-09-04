# This code captures intel realsense d435i
#                    zed2
#                    pmd pico monstar
# added LED relay

import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import pyzed.sl as sl

import argparse
import queue
from pico.roypy_sample_utils import CameraOpener, add_camera_opener_options
from pico.roypy_platform_utils import PlatformHelper
from pico.mylistener import MyListener, process_event_queue

import codecs
import warnings
import serial
import serial.tools.list_ports

#%% parameter setting

wait_second = 60*60

start_LED = 18
finish_LED = 7

LED_delay = 2

#%% data save path

png_ext = '.png'

ir_on_path = './data/realsense/ir_on/'
align_rgb_path = './data/realsense/rgb/'
ir_off_path = './data/realsense/ir_off/'

zed_path = "./data/zed/"

distort_path = './data/pico/distort/'
undistort_path = './data/pico/undistort/'

depth_path = 'depth_'
depth_map_path = 'depth_map_'
left_path = 'left_'
right_path = 'right_'
rgb_path = 'rgb_'
disparity_path = 'disparity_'
confidence_path = 'confidence_'
img_path = 'gray_img_'

#%% LED relay

# Parameter
baudrate = 9600

# Find CH340 port
CH340_port = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'CH340' in p.description
    ]

if not CH340_port:
    raise IOError('No CH340 found')
    
if len(CH340_port) > 1:
    warnings.warn('Multiple CH340 found - using the first')

s = serial.Serial(CH340_port[0],baudrate,stopbits=1)
s.close()

#%% prepare realsense

fps = 30
(W, H) = (1280, 720)

# Create a pipeline for realsense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, W, H, rs.format.z16, fps)
config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, fps)
config.enable_stream(rs.stream.infrared, 1, W, H, rs.format.y8, fps)
config.enable_stream(rs.stream.infrared, 2, W, H, rs.format.y8, fps)

#%% prepare zed2
# Create a ZED camera object
zed = sl.Camera()

# Set config_1uration parameters
input_type = sl.InputType()
if len(sys.argv) >= 2 :
    input_type.set_from_svo_file(sys.argv[1])
init = sl.InitParameters(input_t=input_type)
init.camera_resolution = sl.RESOLUTION.HD1080
init.depth_mode = sl.DEPTH_MODE.ULTRA
init.coordinate_units = sl.UNIT.MILLIMETER
init.depth_minimum_distance = 0.15 #15cm

#%% prepare pico

# Set the available arguments
platformhelper = PlatformHelper()
parser = argparse.ArgumentParser (usage = __doc__)
add_camera_opener_options (parser)
options = parser.parse_args()

opener = CameraOpener (options)

try:
    cam = opener.open_camera ()
except:
    print("could not open pico Camera Interface")
    sys.exit(1)

#%% Streaming loop

while True:

    start_time = time.time()
    timer = time.strftime('%Y%m%d_%H%M%S')
    print('Start capture : ', timer)
    
    current_hour = time.localtime(start_time)[3]
    
    #%% LED on
    
    if current_hour >= start_LED or current_hour <= finish_LED:
    
        # TO close relay (on)
        s.open()
        on = 'A00101A2'
        on_hex = codecs.decode(on, 'hex')
        s.write(on_hex)
        s.close()
        print('LED ON')
        
        # Give time delay
        time.sleep(LED_delay)
        
    else:
        print('Not time to LED ON')

    #%% Start realsense streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    #depth_scale = depth_sensor.get_depth_scale()
    #emitter = depth_sensor.get_option(rs.option.emitter_enabled)
    #print("Depth Scale is: " , depth_scale)
    #print("Emitter = ", emitter)

    # Turns the point cloud laser on or off
    set_emitter = 1 # 0 = off, 1 = on
    depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)

    # Give delay
    time.sleep(2)

    # Wait for a coherent pair of frames
    frames = pipeline.wait_for_frames()

    #%% realsense ir on

    ir_depth_frame = frames.get_depth_frame()
    ir1_frame = frames.get_infrared_frame(1) # Left IR Camera, it allows 0, 1 or no input
    ir2_frame = frames.get_infrared_frame(2) # Right IR camera

    timer = time.strftime('%Y%m%d_%H%M%S')

    ir_depth_image = np.asanyarray(ir_depth_frame.get_data())
    ir1_image = np.asanyarray(ir1_frame.get_data())
    ir2_image = np.asanyarray(ir2_frame.get_data())

    # Render images
    ir_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(ir_depth_image, alpha=0.05), cv2.COLORMAP_JET)

    # Save data
    save_ir_on_depth = ir_on_path + depth_path + timer
    save_ir_on_depth_map = ir_on_path + depth_map_path + timer  + png_ext
    save_ir_on_left = ir_on_path + left_path + timer  + png_ext
    save_ir_on_right = ir_on_path + right_path + timer + png_ext

    np.save(save_ir_on_depth, ir_depth_image)
    cv2.imwrite(save_ir_on_depth_map, ir_depth_map)
    cv2.imwrite(save_ir_on_left, ir1_image)
    cv2.imwrite(save_ir_on_right, ir2_image)
    print('saved realsense ir on data  : ', timer)

    #%% realsense align with rgb

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # frames.get_depth_frame() is a 640x360 depth image
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    timer = time.strftime('%Y%m%d_%H%M%S')

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue

    color_image = np.asanyarray(color_frame.get_data())
    color_depth_image = np.asanyarray(aligned_depth_frame.get_data())

    # Render images
    color_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(color_depth_image, alpha=0.05), cv2.COLORMAP_JET)

    # Save data
    save_rgb = align_rgb_path + rgb_path + timer + png_ext
    save_rgb_depth = align_rgb_path + depth_path + timer
    save_rgb_depth_map = align_rgb_path + depth_map_path + timer + png_ext

    cv2.imwrite(save_rgb, color_image)
    np.save(save_rgb_depth, color_depth_image)
    cv2.imwrite(save_rgb_depth_map, color_depth_map)
    print('saved realsense rgb data    : ', timer)

    #%% realsense ir off

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    #depth_scale = depth_sensor.get_depth_scale()
    #emitter = depth_sensor.get_option(rs.option.emitter_enabled)
    #print("Depth Scale is: " , depth_scale)
    #print("Emitter = ", emitter)

    # Turns the point cloud laser on or off
    set_emitter = 0 # 0 = off, 1 = on
    depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)

    # Give delay
    time.sleep(2)

    # Wait for a coherent pair of frames
    frames = pipeline.wait_for_frames()

    ir_depth_frame = frames.get_depth_frame()
    ir1_frame = frames.get_infrared_frame(1) # Left IR Camera, it allows 0, 1 or no input
    ir2_frame = frames.get_infrared_frame(2) # Right IR camera

    timer = time.strftime('%Y%m%d_%H%M%S')

    ir_off_depth_image = np.asanyarray(ir_depth_frame.get_data())
    ir_off_left_image = np.asanyarray(ir1_frame.get_data())
    ir_off_right_image = np.asanyarray(ir2_frame.get_data())

    # Render images
    ir_off_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(ir_off_depth_image, alpha=0.05), cv2.COLORMAP_JET)

    # Save data
    save_ir_off_depth = ir_off_path + depth_path + timer
    save_ir_off_depth_map = ir_off_path + depth_map_path + timer  + png_ext
    save_ir_off_left = ir_off_path + right_path + timer  + png_ext
    save_ir_off_right = ir_off_path + left_path + timer + png_ext

    np.save(save_ir_off_depth, ir_off_depth_image)
    cv2.imwrite(save_ir_off_depth_map, ir_off_depth_map)
    cv2.imwrite(save_ir_off_left, ir_off_left_image)
    cv2.imwrite(save_ir_off_right, ir_off_right_image)
    print('saved realsense ir off data : ', timer)

    pipeline.stop()

    #%% zed

    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        #exit(1)

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_camera_information().camera_resolution

    err = zed.grab(runtime)
    if err == sl.ERROR_CODE.SUCCESS :

        depth_mat = sl.Mat()
        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU) # Get the depth map
        depth = depth_mat.get_data()

        disparity_mat = sl.Mat()
        zed.retrieve_measure(disparity_mat, sl.MEASURE.DISPARITY, sl.MEM.CPU)
        disparity = disparity_mat.get_data()

        confidence_mat = sl.Mat()
        zed.retrieve_measure(confidence_mat, sl.MEASURE.CONFIDENCE, sl.MEM.CPU)
        confidence = confidence_mat.get_data()

        depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
        zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
        depth_image_ocv = depth_image_zed.get_data()

        image_sl_left = sl.Mat()
        zed.retrieve_image(image_sl_left, sl.VIEW.LEFT)
        image_cv_left = image_sl_left.get_data()

        image_sl_right = sl.Mat()
        zed.retrieve_image(image_sl_right, sl.VIEW.RIGHT)
        image_cv_right = image_sl_right.get_data()

        # Save data
        timer = time.strftime('%Y%m%d_%H%M%S')
        save_zed_depth = zed_path + depth_path + timer
        save_zed_disparity = zed_path + disparity_path + timer
        save_zed_confidence = zed_path + confidence_path + timer
        save_zed_depth_map = zed_path+ depth_map_path + timer  + png_ext
        save_zed_left = zed_path + left_path + timer  + png_ext
        save_zed_right = zed_path + right_path + timer + png_ext

        np.save(save_zed_depth, depth)
        np.save(save_zed_disparity, disparity)
        np.save(save_zed_confidence, confidence)
        cv2.imwrite(save_zed_depth_map, depth_image_ocv)
        cv2.imwrite(save_zed_left, image_cv_left)
        cv2.imwrite(save_zed_right, image_cv_right)
        print('saved zed2 data             : ', timer)

    zed.close()

    #%% pico

    q = queue.Queue()
    l = MyListener(q)
    cam.registerDataListener(l)
    cam.startCapture()

    lensP = cam.getLensParameters()
    l.setLensParameters(lensP)

    (Distort_gzImage8, Distort_grayImage8, Undistort_gzImage8, Undistort_grayImage8) = process_event_queue (q, l)

    timer = time.strftime('%Y%m%d_%H%M%S')

    # Render images
    Distort_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(Distort_gzImage8, alpha=0.7), cv2.COLORMAP_JET)

    #Save data
    save_distort_img = distort_path + img_path + timer + png_ext
    save_distort_depth = distort_path + depth_path + timer
    save_distort_depth_map = distort_path + depth_map_path + timer + png_ext

    cv2.imwrite(save_distort_img, Distort_gzImage8)
    np.save(save_distort_depth, Distort_grayImage8)
    cv2.imwrite(save_distort_depth_map, Distort_depth_map)

    # Render images
    Undistort_depth_map = cv2.applyColorMap(cv2.convertScaleAbs(Undistort_gzImage8, alpha=0.7), cv2.COLORMAP_JET)

    # Save data
    save_undistort_img = undistort_path + img_path + timer + png_ext
    save_undistort_depth = undistort_path + depth_path + timer
    save_undistort_depth_map = undistort_path + depth_map_path + timer + png_ext

    cv2.imwrite(save_undistort_img, Undistort_gzImage8)
    np.save(save_undistort_depth, Undistort_grayImage8)
    cv2.imwrite(save_undistort_depth_map, Undistort_depth_map)
    print('saved pico monstar data     : ', timer)

    cam.stopCapture()
    
    #%% LED off
    
    if current_hour >= start_LED or current_hour <= finish_LED:
    
        # Give time delay
        time.sleep(LED_delay)
        
        # To open relay (off)
        s.open()
        off = 'A00100A1'
        off_hex = codecs.decode(off, 'hex')
        s.write(off_hex)
        s.close()
        print('LED OFF\n')
        
    else:
        print('\n')

    #%% Show image
    
    """
    import matplotlib.pyplot as plt
    
    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax1.imshow(ir1_image,cmap='gray')
    ax1.axis('off')

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.imshow(ir2_image,cmap='gray')
    ax2.axis('off')

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.imshow(cv2.cvtColor(ir_depth_map, cv2.COLOR_BGR2RGB))
    ax3.axis('off')

    plt.show()

    fig = plt.figure()

    ax1 = fig.add_subplot(2, 1, 1)
    ax1.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
    ax1.axis('off')

    ax1 = fig.add_subplot(2, 1, 2)
    ax1.imshow(cv2.cvtColor(color_depth_map, cv2.COLOR_BGR2RGB))
    ax1.axis('off')

    plt.show()

    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax1.imshow(ir_off_left_image,cmap='gray')
    ax1.axis('off')

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.imshow(ir_off_right_image,cmap='gray')
    ax2.axis('off')

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.imshow(cv2.cvtColor(ir_off_depth_map, cv2.COLOR_BGR2RGB))
    ax3.axis('off')

    plt.show()

    fig = plt.figure()

    ax1 = fig.add_subplot(5, 1, 1)
    ax1.imshow(cv2.cvtColor(image_cv_left, cv2.COLOR_BGR2RGB))
    ax1.axis('off')

    ax2 = fig.add_subplot(5, 1, 2)
    ax2.imshow(cv2.cvtColor(image_cv_right, cv2.COLOR_BGR2RGB))
    ax2.axis('off')

    ax3 = fig.add_subplot(5, 1, 3)
    ax3.imshow(depth_image_ocv)
    ax3.axis('off')

    ax4 = fig.add_subplot(5, 1, 4)
    ax4.imshow(disparity, cmap='gray')
    ax4.axis('off')

    ax5 = fig.add_subplot(5, 1, 5)
    ax5.imshow(confidence, cmap='gray')
    ax5.axis('off')

    plt.show()

    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax1.imshow(Distort_grayImage8,cmap='gray')
    ax1.axis('off')

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.imshow(Distort_gzImage8,cmap='gray')
    ax2.axis('off')

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.imshow(cv2.cvtColor(Distort_depth_map, cv2.COLOR_BGR2RGB))
    ax3.axis('off')

    plt.show()

    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax1.imshow(Undistort_grayImage8,cmap='gray')
    ax1.axis('off')

    ax2 = fig.add_subplot(3, 1, 2)
    ax2.imshow(Undistort_gzImage8,cmap='gray')
    ax2.axis('off')

    ax3 = fig.add_subplot(3, 1, 3)
    ax3.imshow(cv2.cvtColor(Undistort_depth_map, cv2.COLOR_BGR2RGB))
    ax3.axis('off')

    plt.show()
    """


    #%% Give delay

    running_time = time.time() - start_time

    while True:
        if running_time <= wait_second:
            running_time = time.time() - start_time
        else:
            running_time = running_time - wait_second
            break

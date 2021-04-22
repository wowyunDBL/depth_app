## 0319
1. connected in USB 2.1
```
[ INFO] [1616143122.262192915]: Device with physical ID 1-2.1-23 was found.
[ INFO] [1616143122.262256711]: Device with name Intel RealSense D435 was found.
[ INFO] [1616143122.264078117]: Device with port number 1-2.1 was found.
[ INFO] [1616143122.264216074]: Device USB type: 2.1
[ WARN] [1616143122.264292838]: Device 832412070163 is connected using a 2.1 port. Reduced performance is expected.
[ INFO] [1616143122.264484801]: Resetting device...
```

```
[ WARN] [1616395557.449316472]: No stream match for pointcloud chosen texture Process - Color
ERROR [546584908160] (uvc-streamer.cpp:106) uvc streamer watchdog triggered on endpoint: 130
\# TF tree issue
Message removed because it is too old (frame=[camera_depth_optical_frame], stamp=[1616232300.882767916])
```

2. file log:
```
├── camera-realsense2_camera-2-stdout.log  \\process died log  
├── known_hosts \\remove this when cannot ssh, directly detele  
├── lab-0319-short.bag  
├── lab-3.bag  
├── lab-.bag  
├── round1.bag \\go straight  
├── round2.bag \\turn 90 degree  
├── round3.bag \\turn 2 90 degree  
└── round4.bag \\with video   
 
```

3. compressed depth rosmsg cannot directly use image_transport of raw:
```
[ WARN] [1616234838.896853954]: OGRE EXCEPTION(2:InvalidParametersException): Stream size does not match calculated image size in Image::loadRawData at /build/ogre-1.9-mqY1wq/ogre-1.9-1.9.0+dfsg1/OgreMain/src/OgreImage.cpp (line 283)
[ERROR] [1616234838.896968606]: Error loading image: OGRE EXCEPTION(2:InvalidParametersException): Stream size does not match calculated image size in Image::loadRawData at /build/ogre-1.9-mqY1wq/ogre-1.9-1.9.0+dfsg1/OgreMain/src/OgreImage.cpp (line 283)
```


## 0327
[Using Camera Tracking]
https://www.stereolabs.com/docs/tutorials/positional-tracking/
```
/depthapp/scripts
.
├── cbDepth.py \\callback depth and record depth in .csv  
└── check_depth.py \\depth post-processing  

/depthapp/jupyterNotebook
.
├── test_image_data.ipynb \\
└── tree_subtr_npy.ipynb \\calculate HOG and standard data pointer

```

## 0411
### extract trunk and do statistic analysis and HOG
### map into 3D coordinate
https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0?fbclid=IwAR3gogVZe824YUps88Dzp02AN_XzEm1BDb0UbmzfoYvn1qDFb7KzbIz9twU#intrinsic-camera-parameters  
http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html  
http://nicolas.burrus.name/index.php/Research/KinectCalibration  

## 0422
### write myUtils.py file
to collect useful function in a file
### Check the Python version in the script: sys
import sys
print(sys.version)

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

## 0510
### Subtract tree truck by fusing Depth and Height information
```
.
├── ground_subtr_npy.ipynb \\Detailed flow
├── low_pass.ipynb \\temporal filter
├── mark_obj.ipynb \\test the single depth image object
├── transformation.ipynb \\self-made constant-height plain image
└── tree_subtr_npy.ipynb
```

### trunk tracking

```
'''trunk center tracker'''
from collections import deque
pts = deque(maxlen=5)
thickness = int(np.sqrt(5 / float(i + 1)) * 2.5)
cv2.line(npTreeMask_c, pts[i - 1], pts[i], (0, 0, 255), thickness)
```

### low freq filter
```
if len(param_model) == 5:
        flag = True
        print("BD: initialize param ground!")
        param_model = param_model/5
        moving_avg = np.zeros_like(param_model)
        moving_avg[param_model<=0.5]=int(0)
        moving_avg[param_model>0.5]=int(255)
        moving_avg = moving_avg.astype('uint8')
        print(len(param_model))

elif len(param_model) < 5:
	print("BD: not yet initial!")
	moving_avg = npHeight_binary
	param_model = param_model + npHeight_binary
	print(len(param_model))
else: 
        print('BD: '+str(len(param_model))+"initialized")
        alpha=0.2
        param_model = (1-alpha)*param_model+alpha*npHeight_binary
        param_model = param_model.astype('uint8')
        moving_avg = np.zeros_like(param_model)
        moving_avg[param_model<=130]=int(0)
        moving_avg[param_model>130]=int(255)
```
## 0611

### realsense in Gazebo
https://github.com/IntelRealSense/realsense-ros/issues/1769
there also a video
https://github.com/issaiass/realsense2_description

## 0618
roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
sudo apt-get remove ros-noetic-turtlebot-*
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

launch-prefix="gnome-terminal -x
```
# solved (? if only need to publish /tf
xacro: in-order processing became default in ROS Melodic. You can drop the option.
[spawn_urdf-4] process has finished cleanly

# you have to add following section in .launch file
<include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
    <arg name="model" value="$(arg model)" />
</include>
```

## 0625
1. /tf 
rospy.Time(0): constructs a /Time/ instance that has special significance in TF contexts: it will cause /lookupTransform(..)/ and friends to return the latest available data for a specific transform, instead of the data at a specific point in time.
rospy.Time.now(): just returns the current 'wall clock time' (or the simulated clock, depending on whether you have a Clock server running).
2. https://cxx0822.github.io/2020/05/05/gmapping-suan-fa-yuan-li-ji-yuan-dai-ma-jie-xi/
3. https://pojenlai.wordpress.com/2015/07/16/ros-navigation-stack-%E7%B0%A1%E4%BB%8B/


## 0628
### hector_mapping
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch depth_app depth2pc_4_simulation.launch
roslaunch depth_app hector_mower.launch
```

# rs_depthApp

https://blog.miniasp.com/post/2018/05/28/Git-Credential-Howto  

## current command
```
rosbag record /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered

rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"

1. 
rostopic echo /gps/qual
rostopic echo /imu/data
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"

2. roslaunch realsense2_camera rs_camera_1.launch 
 
3. rosbag record /camera/color/image_raw/compressed /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered

rosbag record /camera/color/image_raw/compressed /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /imu_filter/rpy/filtered

 
rosbag record /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps  

rostopic echo /gps/qual
rostopic echo /imu/data
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"

### 18 topics
roslaunch depth_app comp2raw.launch
rosrun rviz rviz -d `rospack find depth_app`/rs.rviz
```

## save compressed  
```
<node pkg="image_transport" type="republish" name="republish" args="compressed in:=/camera/color/image_raw/compressed raw out:=/camera/color/image_raw/image"/>
```

# check package
```
anny@anny-hehe:~$ roscd compressed_
compressed_depth_image_transport/
compressed_image_transport/
\#such as package ros-kinetic-compressed-image-transport

```

## python installation
python -m pip install numpy
python-3.6 -m pip install numpy

## 0314  
git add <file>  
git commit -m ""  
git push -u  
git remote -v //check remote  
git remote add origin https://github.com/wowyunDBL/rs_depthApp.git

rosrun rviz rviz -d `rospack find depth_app`/rs.rviz  
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true  
rosrun rqt_reconfigure rqt_reconfigure

rosbag record /camera/color/image_raw /camera/depth/image_rect_raw /camera/depth/color/points /camera/aligned_depth_to_color/image_raw /tf_static /tf  

rosrun depthApp_node test_cbImg.py True  


## 0319 
```
ping 192.168.0.1xx
ssh ursrobot@192.168.0.1xx

roslaunch realsense2_camera rs_camera.launch
roslaunch depth_app comp2raw.launch

rosbag record /camera/color/image_raw/compressed /camera/depth/image_rect_raw/compressed /camera/depth/color/points /camera/aligned_depth_to_color/image_raw/compressed /tf_static /tf /imu/data /husky_velocity_controller/odom /navsat/fix /clock

rosrun rosbag_to_csv rosbag_to_csv.py
```
## 0429
```
import myUtils

# wanna save full data not like 0,...,0
np.set_printoptions(threshold=sys.maxsize)

# image in this format: image must be converted to 8-bit per pixel
npHeight_color = npHeight_color.astype('uint8')

# in Jupyter notebook
# if want to expand list
print(*a)
```

## 0531
```
roslaunch depth_app depth2pc.launch
roslaunch depth_app gmapping_mower.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

rosrun map_server map_saver -f ~/map
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
rosbag info two_trees.bag
```
### How to set scan_height?
DepthImageToLaserScan.h

### mktime
```
>>> t = 1618892029713
>>> time.strftime("%a %d %b %Y %H:%M:%S GMT", time.gmtime(t / 1000.0))
'Tue 20 Apr 2021 04:13:49 GMT'

>>> rosparam get use_sim_time

<node pkg="tf" type="static_transform_publisher" name="cam_link_broadcaster" args="0 0 0.36 0 0 0 base_footprint camera_link 100" />
<node pkg="tf" type="static_transform_publisher" name="cam_link_broadcaster" args="0 0 0.36 0 0 0 base_link camera_link 100" />

```
### need install
```
sudo apt-get install ros-noetic-gmapping ros-noetic-navigation 
```

### gazebo realsense
https://github.com/issaiass/realsense_gazebo_plugin

### check if installed
rospack list-names

## 0625
```
roslaunch depth_app depth2pc.launch

```

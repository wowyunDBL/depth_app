# depth_app

https://blog.miniasp.com/post/2018/05/28/Git-Credential-Howto  

## current command
```
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"

rosbag record /camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered /imu/mag

rosbag record /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered /imu/mag
```

## some utils
```
rosrun rviz rviz -d `rospack find depth_app`/rs.rviz
rosrun rosbag_to_csv rosbag_to_csv.py
```

## file detail
### /launch
```
.
├── amcl_config_mower.launch
├── amcl_config_mower_simulation.launch
├── amcl_mower.launch
├── amcl_mower_simulation.launch
├── comp2raw.launch     # use image_transport to decompress image
├── depth2pc_4_simulation.launch  # to-laser/to-pointcloud
├── gmapping_mower_4_simulation.launch # mapping
├── hector_mower.launch  # mapping
├── map_server.launch
├── map_traj.launch
└── tf_reframe.launch   # play rosbag and remove tf
```
### /scripts
```
.
├── cbDepthGPS_drawPointCloud.py
├── cbdepth.py
├── cbDepth.py
├── cbDepth_v1.py
├── cbDepth_v1_trunkTrack.py
├── cbGPS_initial.py
├── cbIMU.py
├── check_depth.py
├── depth_observer.py
├── ground_seg.py
├── heightT.py
├── motion_controller.py
├── myUtils.py  # load/save function
├── pileUP.py
├── pointCloud_filter.py

```

## python installation
python -m pip install numpy
python-3.6 -m pip install numpy


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

```
(nano)
roslaunch realsense2_camera combine_camera_laser_hector.launch
rosbag record
# roslaunch realsense2_camera hector_mower.launch

(laptop)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
./demo.sh
rosrun map_server map_saver -f ~/demo
```


rosbag record /camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered

first check rviz()tf
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"
rosservice call /outdoor_waypoint_nav/datum


```
roslaunch depth_app amcl_mower.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch realsense2_camera combine_camera_laptop.launch
```

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"
```


```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch depth_app hector_mower.launch
rosrun map_server map_saver -f ~/demo
```

```
roslaunch depth_app amcl_mower.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch realsense2_camera combine_camera_laptop.launch
```

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start"
```

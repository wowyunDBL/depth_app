## some launch file edit
```
<!-- Map server -->
<arg name="map_file" default="$(find turtlebot3_navigation)/maps/map.yaml"/>
<node pkg="map_server" name="map_server" type="map_server" args="$(arg map_file)"/>
```
```
<!-- Hector_mapping -->
<param name="map_with_known_poses" value="true" /> 
<param name="map_frame"  value="odom" />
<arg name="pub_map_odom_transform" default="false"/>
```
```
# plot
fig, ax = plt.subplots(figsize=(6,6),dpi=100)
plt.grid(True)
plt.axis('equal')
base = plt.gca().transData
rotation = transforms.Affine2D().rotate_deg(135)
plt.plot(transform = rotation + base)
```
## 0628
###
```
roslaunch depth_app depth2pc.launch
roslaunch depth_app gmapping_mower.launch
roslaunch depth_app hector_mower.launch
rosrun map_server map_saver -f ~/odomf_right_hector
rosrun map_server map_server mymap.yaml
```
```
mySubscriber_2_CSV.py
mySubscriber_ploter.py
readCSV_odom.py
readCSV_GPS_modi.py
```
### work log
1. learning GIMP: layers, line
2. debug for the heading issue, the tf shrink
3. implement hector_map with known pose
4. plot accumulate point cloud by tf

### Reference
1. TF extrapolation into the future issue 
https://www.cnblogs.com/cv-pr/p/5298075.html

2. Edit a map generated with gmapping
https://www.youtube.com/watch?v=BfCUfmJLJDY&t=570s

3. How to start a hector_mapping 
https://www.youtube.com/watch?v=j1k2alcd7cA

## 0629 
###
```
<pgm .yaml>
image: /home/ncslaber/mapf_right_gmapping_revised.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### work log
1. Install PlotJuggler
2. Progress report -> crazy comparing different rosbag

2021-06-29-11-06-51.bag, 2021-06-29-11-13-32.bag: front_2 ok
2021-06-29-11-23-08: route-right_1 ok
2021-06-29-11-44-00.bag 2021-06-29-11-54-18: left_2
2021-06-29-11-47-30: navigation test 
navigation_test_round.bag => fail ....

### Reference
1. PlotJuggler: learn the basics
https://slides.com/davidefaconti/introduction-to-plotjuggler

## 0701
### ORB-SLAM
```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin/
mkdir build
cd build/
cmake ..
make

<when cmake fail>
sudo apt install libglew-dev

```

```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
<all make -j to only make>
cd ORB_SLAM2
./build.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS # your own PATH
./build_ros.sh

```

```
roslaunch orb_slam2_ros orb_slam2_d435_rgbd.launch
```
https://www.ncnynl.com/archives/201807/2501.html

<.bashrc>
< laptop_ip=192.168.0.102>
< raspberry pi ip=192.168.0.104>
< jetson nano ip= 192.168.0.100>
export ROS_MASTER_URI=http://192.168.0.104:11311
export ROS_HOSTNAME=192.168.0.102
<etc/hosts>
192.168.0.104 pi3test
192.168.0.100 ursrobot


### Turtlebot test for free space
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_slam turtlebot3_gmapping.launch
roslaunch depth_app depth2pc_4_laser.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

### gmapping mark free space
1. depthimage_to_laserscan: value shouldn't be nan
change Depth.cfg
2. use rosnode laser_filters to set specific value

### work log
two video to compare differnet tf

## 0702
```
rosbag record /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered
```

```
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"
```

```
1. 
rosbag record /camera/color/image_raw/compressed /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /imu_filter/rpy/filtered

2. 
rosbag record /camera/color/camera_info /camera/color/image_raw/compressed /camera/depth/camera_info /camera/depth/image_rect_raw /camera/extrinsics/depth_to_color /camera/realsense2_camera_manager/bond /camera/rgb_camera/auto_exposure_roi/parameter_descriptions /camera/rgb_camera/auto_exposure_roi/parameter_updates /camera/rgb_camera/parameter_descriptions /camera/rgb_camera/parameter_updates /camera/stereo_module/auto_exposure_roi/parameter_descriptions /camera/stereo_module/auto_exposure_roi/parameter_updates /camera/stereo_module/parameter_descriptions /camera/stereo_module/parameter_updates /tf_static /tf /imu/data /husky_velocity_controller/odom /outdoor_waypoint_nav/odometry/filtered /outdoor_waypoint_nav/odometry/filtered_map /gps/heading /gps/qual /gps/time_reference /gps/vel /husky_velocity_controller/cmd_vel /navsat/fix /outdoor_waypoint_nav/gps/filtered /outdoor_waypoint_nav/odometry/gps /imu_filter/rpy/filtered
```

```
# gazebo
rosbag record /tf_static /tf /cmd_vel /scan
```

## 0705
```
# 執行 dpkg 指令以確保每個套件都有成功安裝
$ dpkg --get-selections | grep gmapping
ros-kinetic-gmapping				install
ros-kinetic-openslam-gmapping			install
ros-kinetic-slam-gmapping			install
```
```
$ dpkg --get-selections | grep turtlebot3
ros-kinetic-turtlebot3				install
ros-kinetic-turtlebot3-bringup			install
ros-kinetic-turtlebot3-description		install
ros-kinetic-turtlebot3-fake			install
ros-kinetic-turtlebot3-gazebo			install
ros-kinetic-turtlebot3-msgs			install
ros-kinetic-turtlebot3-navigation		install
ros-kinetic-turtlebot3-simulations		install
ros-kinetic-turtlebot3-slam			install
ros-kinetic-turtlebot3-teleop			install

$ apt-cache search ros-kinetic-cartographer
```

### work log
1. draw map with 4.5m/6m free space
2. AMCL
3. gazebo setting
4. understand why goes like this (check paper note)
5. watch ros log

### new launch file for different case
```
```


### ref
1. realsense-ros gazebo plugin
https://github.com/pal-robotics/realsense_gazebo_plugin/issues/7

## 0706
1. use <!-- move_base --> to set initla pose

### install RTAB map
```
# first install
sudo apt-get install ros-kinetic-rtabmap-ros
# 
```

```
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" (approx_sync:=false)

# for localization mode
roslaunch rtabmap_ros rtabmap.launch
```

## 0708
1. gazebo framework and implement return /scan in nano
```
<bug>
sudo apt install libopencv3.2
```
2. understand parameters of
```
<gmapping>
param name="particles" value="1" .... particles in the filer (default:30, which slows
down the updating procedure)
param name="map_update_interval" value="0.1" .... updates in sec
```

```
<amcl> (ref: https://www.twblogs.net/a/5d5ecfcbbd9eee5327fdca4e)
odom_alpha1: 机器人旋转分量中的旋转噪音
```
3. gmapping /entrophy: reflecting the degree of dispersion of robot pose estimation.
4. cartographer
```
sudo apt-get install ros-kinetic-cartographer ros-kinetic-cartographer-ros ros-kinetic-cartographer-ros-msgs ros-kinetic-cartographer-rviz
sudo apt-get remove ros-kinetic-cartographer ros-kinetic-cartographer-ros ros-kinetic-cartographer-ros-msgs ros-kinetic-cartographer-rviz
```

## 0709
1. compare usb 3.2 and 2.1: 2Hz/3Hz
2. return /scan and run gmapping
3. install teleop_twist_keyboard and cartographer
```
rosservice type /outdoor_waypoint_nav/datum => robot_localization/SetDatum
rosservice call /outdoor_waypoint_nav/datum 
"geo_pose: 
 position: 
   latitude: 25.0183 
   longitude: 121.545 
   altitude: 0.0 
 orientation:
   x: 0.0
   y: 0.0
   z: 0.0
   w: 1.0"
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=husky_velocity_controller/cmd_vel _repeat_rate:=10.0

roslaunch realsense2_camera rs_aligned_depth.launch
roslaunch realsense2_camera depth2pc.launch

## 0710 
I have install for 2 days and finally successed!
Note: dependency-> lua, prohub ,,, (and restart might be a good choice)
ref: https://zhuanlan.zhihu.com/p/345761455
```
# run test
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```

```
# gmapping log

Scan Matching Failed, using odometry. Likelihood=-227.64
lp:3.04572 -3.50232 -0.682254
op:3.22058 -3.66675 -0.673163
Average Scan Matching Score=46.4972
neff= 13.9819
*************RESAMPLE***************
Deleting Nodes: 0 4 6 8 9 11 12 14 17 20 21 25 27 28 Done
Deleting old particles...Done
Copying Particles and  Registering  scans... Done
update frame 146
update ld=0.208254 ad=0.0928452
Laser Pose= 3.36824 -3.81098 -0.592377
m_count 37
Scan Matching Failed, using odometry. Likelihood=2.52962e-320
lp:3.22058 -3.66675 -0.673163
op:3.36824 -3.81098 -0.592377
Average Scan Matching Score=48.6681
neff= 24.4349
Registering Scans:Done
update frame 150
update ld=0.236013 ad=0.0679503
Laser Pose= 3.55729 -3.93223 -0.618068
m_count 38
Average Scan Matching Score=58.4531


```

```
roslaunch depth_app map_traj.launch
rostopic pub syscommand std_msgs/String "savegeotiff"
```

## 0712
1. simulation in Gazebo (transform_publish_period: 0.)
2. run in MD601 (6.5m/10m)
3. run in outdoor library
4. remove frames
5. hector_trajectory_server

```
# in my computer
<check /etc/hosts>
roslaunch depth_app gmapping_mower.launch
roslaunch depth_app map_traj.launch
rostopic pub syscommand std_msgs/String "savegeotiff"
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```
# in nano
<check /etc/hosts>
rosservice call /outdoor_waypoint_nav/datum "geo_pose: position: latitude: 25.0172  longitude: 121.542 altitude: 0.0 orientation: x: 0.0 y: 0.0 z: 0.0 w: 1.0"
rostopic pub /imu_filter/calib_comp/calib_request std_msgs/UInt8 "data: 1"
roslaunch realsense2_camera rs_align.launch
roslaunch realsense2_camera depth2pc.launch
```

## 0713
1. roslaunch depth_app tf_reframe.launch => remove map_yun
2. change gmapping field of view
3. finally understand hector_traj: 
4. depth_image replace of align_depth: is it ok to reproduce?
5. what is nodelet: 

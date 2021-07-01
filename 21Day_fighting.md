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
roslaunch turtlebot3_slam turtlebot3_gmapping.launch
roslaunch depth_app depth2pc_4_laser.launch

### gmapping mark free space
1. depthimage_to_laserscan: value shouldn't be nan
change Depth.cfg
2. use rosnode laser_filters to set specific value

### work log
two video to compare differnet tf

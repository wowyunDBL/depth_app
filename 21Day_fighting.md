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

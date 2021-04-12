import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from PIL import Image
import cv2
from matplotlib.ticker import LinearLocator
import matplotlib.ticker as ticker
import pyrealsense2 as rs
import statistics as sta

dnp = []

with open('/home/ncslaber/realSense/catkin_ws/src/depth_app/scripts/output.csv', 'r') as csvfile:
  rows = csv.reader(csvfile, delimiter=',')
  for row in rows:
    dnp.append(row)

dnp = np.asanyarray(dnp) # [0:480]
dnp = dnp.astype('uint16')
print(dnp.shape)
print(dnp.dtype)

depth_filtered = cv2.convertScaleAbs(dnp, alpha=0.04)
print(depth_filtered.shape)

hog = np.zeros(256)
for i in range(480):
    for j in range(640):
        index = depth_filtered[i][j]
        hog[index] = hog[index]+1
hog[255]=0
fig, ax = plt.subplots()
'''
colors = cm.rainbow(np.linspace(1, 0, 256))
for i in range(256):
    plt.plot(i, hog[i], color=colors[i])
'''    
plt.legend(loc='upper right', fontsize=16)
# plt.xticks(range(0,70000,10000))
plt.xlabel('gray intensity',fontsize=16)
plt.ylabel('number of pixel',fontsize=16)
plt.xlim(-10,265)
# plt.ylim(0,500)
plt.title('after compressing', fontsize=16)
# plt.show()

# cv2.imshow('depth filtered',depth_filtered)
# cv2.waitKey(0)

aryMedian = np.zeros(10)
init = False
k=0
height, width = depth_filtered.shape

depth_filtered_copy = np.copy(depth_filtered)
record_mean = []
record_diff = []
# ttmp = depth_filtered_copy[int(480-(i+1)-k*10)]
# ttmp = ttmp[:350]
while(1):
    for i in range(10):
        tmp = depth_filtered_copy[int(480-(i+1)-k*10)][depth_filtered_copy[int(480-(i+1)-k*10)] != 0]
        #tmp = tmp[tmp != 255]
        acc = 0
        for j in range(len(tmp)):
            acc = acc+tmp[j]
        aryMedian[i] = acc/len(tmp) #
        #print(tmp)
    median_area_now = (sta.mean(aryMedian))
    
    print(aryMedian)
    print(median_area_now)
    print(k)
    record_mean.append(median_area_now)
    if k>50:
        break
    if init == False:
        init = True
        median_area_past = median_area_now
        continue
    if np.abs(median_area_past-median_area_now) > 25 and median_area_past-median_area_now !=0:
        print("here")
        print(median_area_now)
        print(median_area_past-median_area_now)
        print(median_area_past)
        record_diff.append(median_area_past-median_area_now)
        mask = np.zeros_like(depth_filtered)
        region_of_interest_vertices = [(0, 0),(0, height-k*10),(width, height-k*10),(width, 0)]
        cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
        masked_img = cv2.bitwise_and(depth_filtered, mask)
        # masked_img = cv2.cvtColor(masked_img, cv2.COLOR_GRAY2BGR)
        cv2.putText(masked_img, #numpy array on which text is written
            str(median_area_now), #text
            (40+25,height-k*10+15), #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            0.9, #font size
            (255,0,0), #font color
            3)
        cv2.imshow('threshold1', masked_img)
        cv2.waitKey(0)
        break
    else:
        mask = np.zeros_like(depth_filtered)
        region_of_interest_vertices = [(0, 0),(0, height-k*10),(width, height-k*10),(width, 0)]
        cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
        masked_img = cv2.bitwise_and(depth_filtered, mask)
        # masked_img = cv2.cvtColor(masked_img, cv2.COLOR_GRAY2BGR)
        cv2.putText(masked_img, #numpy array on which text is written
            str(median_area_now), #text
            (40+25,height-k*10+15), #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            0.9, #font size
            (255,0,0), #font color
            3)
        cv2.imshow('threshold1', masked_img)
        cv2.waitKey(1)
        record_diff.append(median_area_past-median_area_now)
        median_area_past = median_area_now
        k = k+1
'''
plt.scatter(range(len(record_mean)),record_mean, c='r')
plt.plot(record_mean )
plt.xlabel('pixel height',fontsize=16)
plt.ylabel('gray intensity',fontsize=16)
plt.title('average depth (interval=10pixel)',fontsize=16)
plt.xlim(-5,50)
plt.show()
plt.scatter(range(len(record_diff)),record_diff, c='r')
plt.plot(record_diff)
plt.xlabel('pixel height',fontsize=16)
plt.ylabel('difference',fontsize=16)
plt.title('difference depth (interval=10pixel)',fontsize=16)
plt.ylim(-30,30)
plt.show()
'''
ret, depth_filtered_copy = cv2.threshold(depth_filtered_copy, 230, 255, cv2.THRESH_TOZERO_INV)
cv2.imshow('threshold3', depth_filtered_copy)
cv2.waitKey(0)
mask = np.zeros_like(depth_filtered_copy)   
region_of_interest_vertices = [(0, 0),(0, height-17*10),(width, height-17*10),(width, 0)] #Let k=17
cv2.fillPoly(mask, np.array([region_of_interest_vertices], np.int32), 255)
masked_img = cv2.bitwise_and(depth_filtered_copy, mask)
cv2.imshow('threshold3', masked_img)
cv2.waitKey(0)
ret, masked_img = cv2.threshold(masked_img, 170, 255, cv2.THRESH_BINARY)
cv2.imshow('threshold4', masked_img)
cv2.waitKey(0)

depth_filtered_copy1 = np.copy(depth_filtered)
_,contours, hierarchy = cv2.findContours(masked_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
cv2.drawContours(masked_img, contours,-1,(0,0,0),8)  # contour is a list
cv2.imshow('threshold5', masked_img)
cv2.waitKey(0)
#masked_img = cv2.bitwise_and(masked_img, )
accu_depth = 0
accu_amount = 0
tree_depth = []
for i in range(masked_img.shape[0]):
    for j in range(masked_img.shape[1]):
        if masked_img[i][j]!=0:
            accu_depth = accu_depth + dnp[i][j]
            tree_depth.append(dnp[i][j])
            accu_amount = accu_amount + 1
        else: 
            tree_depth.append(7000)
tree_depth = np.asanyarray(tree_depth)
tree_depth = np.reshape(tree_depth, (480,-1))
depth_filtered_copy1_c = cv2.cvtColor(depth_filtered_copy1, cv2.COLOR_GRAY2BGR)
cv2.putText(depth_filtered_copy1_c, #numpy array on which text is written
            str(accu_depth/accu_amount), #text
            (450+35,200+15), #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            0.9, #font size
            (0,0,0), #font color
            3) #font stroke
cv2.imshow('t', depth_filtered_copy1_c)
cv2.waitKey(0)

fig3, ax3 = plt.subplots(subplot_kw={"projection": "3d"})
y = np.linspace(0,479,480) # data of [:x] column
x = np.linspace(0,639,640) # data of [x:] row
X,Y = np.meshgrid(x,y)
print(tree_depth.shape)
initial_cmap = cm.get_cmap('rainbow')
reversed_cmap=initial_cmap.reversed()
surf = ax3.plot_surface(X, Y, tree_depth, cmap=reversed_cmap) # , cmap=reversed_cmap)
# boundRect = cv2.boundingRect(contours)
plt.title('Depth tree', fontsize='15')
plt.xlabel('X[pixel]', fontsize='15')
plt.ylabel('Y[pixel]', fontsize='15')
fig3.colorbar(surf, shrink=0.5, aspect=5)
plt.show()



'''
# ret, out = cv2.threshold(depth_filtered, 180, 255, cv2.THRESH_TOZERO)
depth_filtered_c = cv2.cvtColor(depth_filtered, cv2.COLOR_GRAY2BGR)
cv2.circle(depth_filtered_c, (450,200), 8, (255,0,0),-1)
cv2.putText(depth_filtered_c, #numpy array on which text is written
            str(depth_filtered[200][450]), #text
            (450+15,200+15), #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            0.5, #font size
            (255,0,0), #font color
            3) #font stroke
cv2.circle(depth_filtered_c, (250,450), 8, (0,255,0),-1)
cv2.putText(depth_filtered_c, #numpy array on which text is written
            str(depth_filtered[450][200]), #text
            (250+15,450+15), #position at which writing has to start
            cv2.FONT_HERSHEY_SIMPLEX, #font family
            0.5, #font size
            (0,255,0), #font color
            3) #font stroke
cv2.imshow('dot depth', depth_filtered_c)
cv2.waitKey(0)
'''
# depth_colormap = cv2.applyColorMap(depth_filtered, cv2.COLORMAP_JET)

# cv2.circle(dnp, (10,10),15,(255,0,0),-1)
# cv2.imshow("raw depth", np.uint8(dnp))
# cv2.waitKey(0)


fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
y = np.linspace(0,479,480) # data of [:x] column
x = np.linspace(0,639,640) # data of [x:] row
X,Y = np.meshgrid(x,y)
# colors = cm.rainbow(np.linspace(1, 0, 65535))
ax.zaxis.set_major_locator(LinearLocator(5))
# ax.zaxis.set_major_formatter('{x:.02f}')

ax.zaxis.set_major_formatter(ticker.FormatStrFormatter('%d'))

initial_cmap = cm.get_cmap('rainbow')
reversed_cmap=initial_cmap.reversed()
# ax.set_zlim(0,65535)
surf = ax.plot_surface(X, Y, depth_filtered, cmap=reversed_cmap)
# ax.invert_zaxis()
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.title('Depth filtered', fontsize='15')
plt.xlabel('X[pixel]', fontsize='15')

plt.ylabel('Y[pixel]', fontsize='15')
ax.set_zlabel('depth[mm]', fontsize='15')
# get a tick and will position things next to the last one
ticklab = ax.zaxis.get_ticklabels()[0]
trans = ticklab.get_transform()
ax.zaxis.set_label_coords(50000, 0, transform=trans)

# plt.show()
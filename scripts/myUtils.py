#!usr/bin/env python
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms

file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/"

def write2CSV(i, data):
    with open(file_path + "s_2/" + str(i) + '.csv', 'a+') as csvfile: # or w
        writer = csv.writer(csvfile)
        writer.writerows(np.reshape(data,(1,-1)))

def write2CSV_column(i, data):
    with open(file_path + "s_2/" + str(i) + '.csv', 'a+') as csvfile: # or w
        writer = csv.writer(csvfile)
        writer.writerow(np.reshape(data,(1,-1)))

def write2Numpy():
    np.save(file_path + 'two_tree', image)

def read4CSV():
    npDepth_mean = []
    npDepth_median = []
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/s_2/"
    with open(file_path + 'statistics.csv', 'r') as csvfile:
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            npDepth_mean.append(row[0])
            npDepth_median.append(row[1])
    npDepth_mean = np.asanyarray(npDepth_mean)
    npDepth_median = np.asanyarray(npDepth_median)
    # print(npDepth.dtype) # U5 means string length < 5
    # print(npDepth.shape)
    npDepth_mean = npDepth_mean.astype('float32')
    npDepth_median = npDepth_median.astype('float32')
    fig, ax = plt.subplots()
    plt.plot(npDepth_mean, 'go-', label='mean')
    plt.plot(npDepth_median, 'bo-', label='median')
    xticks = range(0,43,10)
    ax.xaxis.set_ticks(xticks)  
    plt.title('statistics')
    plt.xlabel('frame')
    plt.ylabel('height [mm]')
    plt.axhline(5, label='5')
    plt.legend()
    plt.show()

def read4CSV1():
    npDepth_mean = []
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/p_1_45/"
    with open(file_path + 'histogram.csv', 'r') as csvfile:
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            npDepth_mean.append(row)
    npDepth_mean = np.asanyarray(npDepth_mean)
    # print(npDepth.dtype) # U5 means string length < 5
    # print(npDepth.shape)
    npDepth_mean = npDepth_mean.astype('int32')
    fig, ax = plt.subplots()
    plt.plot(npDepth_mean[:10], 'go-', label='histogram')
    plt.legend()
    plt.show()

def readNumpy():
    imu_roll = np.load("/home/ncslaber/imu.npy")
    plt.title('imu data')
    plt.xlabel('msgs')
    plt.ylabel('pitch (rad)')
    plt.plot(imu_roll, linewidth = 2.0)
    plt.show()


def CVshow():
    cv2.imshow("filtered", npDepthF_color)
    cv2.waitKey(0)
    # cv2.destroyAllWindows()
    cv2.destroyWindow("raw")

def array_broadcasting():
    a = np.array([[1,2,3,4,5],[1,2,2,2,5],[4,4,3,2,5]])
    a[np.logical_and(a>3, a!=5)]=0

def matplot_transform():
    fig, ax = plt.subplots(figsize=(10,10))
    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(135)
    plt.scatter([0],[0],c = 'r',transform = rot + base)
    plt.scatter([0],[1],c = 'b',transform = rot + base)
    plt.scatter([1],[1],c = 'g',transform = rot + base)
    plt.scatter([1],[0],c = 'orange',transform = rot + base)
    plt.xlim(-2,2)
    plt.ylim(-2,2)
    #ax.axis('equal')
    plt.show()

def HOG():
    HOG_all = np.zeros(65536)
    for i in range(480):
        for j in range(640):
            index = npDepth[i][j]
            HOG_all[index] = HOG_all[index]+1
    plt.plot(HOG_all)

def cvUtils():
    cv2.line(npDepth_copy, (0,380),(640,380),(0,0,0),3)

def plot3D_color_surface():
    '''plot 3D color surface'''
    # %matplotlib qt
    fig3, ax3 = plt.subplots(subplot_kw={"projection": "3d"})
    y = np.linspace(0,479,480) # data of [:x] column
    x = np.linspace(0,639,640) # data of [x:] row
    X,Y = np.meshgrid(x,y)
    initial_cmap = cm.get_cmap('rainbow')
    surf = ax3.plot_surface(X, Y, npDepthF, cmap=initial_cmap)
    plt.title('Depth tree', fontsize='15')
    plt.xlabel('X[pixel]', fontsize='15')
    plt.ylabel('Y[pixel]', fontsize='15')
    fig3.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()

if __name__ == '__main__':
    readNumpy()
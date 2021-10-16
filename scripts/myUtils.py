#!usr/bin/env python
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms
import matplotlib.cm as cm

file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/sta/"

def write2CSV(file_name, data):
    with open(file_name + '.csv', 'a+') as csvfile: # or w
        writer = csv.writer(csvfile)
        writer.writerows(np.reshape(data,(480,640)))

def write2CSV_column(file_name, data):
    with open(file_name + '.csv', 'a+') as csvfile: # or w
        writer = csv.writer(csvfile)
        writer.writerow(np.reshape(data,(1,-1)))

def write2Numpy(image):
    np.save(file_path + 'two_tree', image)

def read4CSV():

    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/s_2/"
    with open(file_path + 'statistics.csv', 'r') as csvfile:
        data = list( csv.reader(csvfile, delimiter=',') )
        npDepth_mean = np.array(data)[:,0].astype('float32') # dtype: U5 means string length < 5
        npDepth_median = np.array(data)[:,1].astype('float32')
    
    fig, ax = plt.subplots(dpi=200)
    plt.plot(npDepth_mean, 'go-', label='mean')
    plt.plot(npDepth_median, 'bo-', label='median')
    # xticks = range(0,43,10)
    # ax.xaxis.set_ticks(xticks)  
    plt.title('statistics')
    plt.xlabel('frame')
    plt.ylabel('height [mm]')
    plt.axhline(5, label='5')
    plt.legend()
    plt.show()

def read4Numpy():
    imu_roll = np.load("/home/ncslaber/imu.npy")

def CVshow(colorimg):
    import cv2
    cv2.imshow("filtered", colorimg)
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
    ax.axis('equal')
    plt.show()

def HOG(npDepth):
    HOG_all = np.zeros(65536)
    for i in range(480):
        for j in range(640):
            index = npDepth[i][j]
            HOG_all[index] = HOG_all[index]+1
    plt.plot(HOG_all)

def plot3D_color_surface(npDepth):
    '''plot 3D color surface'''
    # %matplotlib qt
    fig3, ax3 = plt.subplots(subplot_kw={"projection": "3d"})
    y = np.linspace(0,479,480) # data of [:x] column
    x = np.linspace(0,639,640) # data of [x:] row
    X,Y = np.meshgrid(x,y)
    initial_cmap = cm.get_cmap('rainbow')
    surf = ax3.plot_surface(X, Y, npDepth, cmap=initial_cmap)
    plt.title('Depth tree', fontsize='15')
    plt.xlabel('X[pixel]', fontsize='15')
    plt.ylabel('Y[pixel]', fontsize='15')
    fig3.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()


if __name__ == '__main__':

    fDepth = np.load('/home/ncslaber/110-1/211009_allLibrary/front-right/syn_rosbag/depth/10.npy')
    write2CSV('/home/ncslaber/110-1/211009_allLibrary/front-right/syn_rosbag/depth-10',fDepth)
    pass
#!usr/bin/env python
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms

file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/sta/"

def write2CSV(i, data):
    with open(file_path + "sta/" + str(i) + '.csv', 'a+') as csvfile: # or w
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
    # xticks = range(0,43,10)
    # ax.xaxis.set_ticks(xticks)  
    plt.title('statistics')
    plt.xlabel('frame')
    plt.ylabel('height [mm]')
    plt.axhline(5, label='5')
    plt.legend()
    plt.show()

def read4CSV1():
    npDepth_mean = []
    npDepth_median = []
    npDepth_std = []
    file_path = "/home/ncslaber/109-2/tree_experiment/npy_depth/sta/"
    with open(file_path + 'statistics1.csv', 'r') as csvfile:
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            npDepth_mean.append(row[0])
            npDepth_median.append(row[1])
            npDepth_std.append(row[2])
    npDepth_mean = np.asanyarray(npDepth_mean)
    npDepth_median = np.asanyarray(npDepth_median)
    npDepth_std = np.asanyarray(npDepth_std)
    # print(npDepth.dtype) # U5 means string length < 5
    # print(npDepth.shape)
    npDepth_mean = npDepth_mean.astype('float32')
    npDepth_median = npDepth_median.astype('float32')
    npDepth_std = npDepth_std.astype('float32')
    fig, ax = plt.subplots(dpi=200)
    plt.plot(npDepth_mean, 'go-', label='mean', linewidth=2)
    plt.plot(npDepth_median, 'ro-',label='median')
    plt.plot(npDepth_std, 'bo-', label='std')
    # ax.axes.yaxis.set_visible(False)
    plt.title('Tree statistics', fontsize=25)
    plt.legend(fontsize=20)
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

def timer():
    pass
    # import time
    # start_time = time.time()
    # print("----------------world coord: "+str(time.time()-start_time))

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

def drawGPS():
    path = '/home/anny/109-2/0618_mapExperiment/oneTree_MD/'
    bags = [
            path + '2021-06-13-11-31-48-navsat-fix.csv',
            ]
     

    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(92)#
    for idx, bag in enumerate(bags):	
        time, lng, lat = readGPS(bag)
        import utm
        _, _, zone, _ = utm.from_latlon(lat[0], lng[0])
        proj = Proj(proj='utm', zone=zone, ellps='WGS84', preserve_units=False)
        ux, uy = proj(lng, lat)

    print(np.asarray(ux).shape)
    colors = cm.rainbow(np.linspace(1, 0, np.asarray(ux).shape[0]))
    plt.scatter(ux, uy, c=colors, transform = rot + base)
    
    ''' print timeStamp every 5 sec 
    for i in range(0,319,50):
        ax.text(ux[i]+0.6, uy[i], str(math.trunc(i/5/60))+":"+str(math.trunc(i/5%60)), fontsize=14, transform = rot + base)
    '''

    fig, ax = plt.subplots(figsize=(18, 18))
    ax.axis('equal')
    #plt.tight_layout()
    plt.grid(True)
    plt.title("GPS", fontsize=25)
    
    plt.yticks(fontsize=15)
    plt.xticks(fontsize=10)
    plt.xlim((-2778355, -2778330))
    plt.ylim((256076, 256092))
    #ax.xaxis.set_major_locator(FixedLocator(np.arange(352883, 352898, 1)))
    ax.ticklabel_format(useOffset=False, style='sci')
    #ax.yaxis.set_major_locator(FixedLocator(np.arange(2.76766**6, 2.76773**6, 0.1)))
    # plt.savefig(path+"test_timeMarker.png")
    
    plt.show()

if __name__ == '__main__':
    read4CSV1()
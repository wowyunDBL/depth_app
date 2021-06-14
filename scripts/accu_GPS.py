#! /usr/bin/env python

'''ros utils'''
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib import transforms
import math
import matplotlib.animation as animation

file_path = "/home/anny/109-2/0618_mapExperiment/oneTree_MD/np_"

PointX_g = []
PointY_g = []
tx_g = np.array([])
ty_g = np.array([])

for i in range(31):#29
    for j in range(0,10,2):
        # file_path = "/home/anny/109-2/0420_treeExperiment/npy/2_gps_npy/np_"
        PointX, PointY, tx, ty = np.load(file_path+str(i)+"_"+str(j)+".npy", allow_pickle=True)
        PointX_g.append(PointX)
        PointY_g.append(PointY)
        tx_g = np.append(tx_g,tx)
        ty_g = np.append(ty_g,ty)

colors = cm.rainbow(np.linspace(1, 0, np.asarray(tx_g).shape[0]))
fig, ax = plt.subplots(figsize=(6,9),dpi=100)
plt.grid(True)
base = plt.gca().transData
rotation = transforms.Affine2D().rotate_deg(92)#92
for i in range(len(tx_g)):
  plt.scatter(PointX_g[i],PointY_g[i], c=colors[i], transform = rotation + base, s=1)
plt.scatter(tx_g, ty_g, c='k', transform = rotation + base, s=5)

plt.xlim((-2.5, 1))
plt.ylim((4, 12))

plt.savefig(file_path+"total.png", dpi=100)

plt.show()

#! /usr/bin/env python

'''ros utils'''
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib import transforms
import math
import matplotlib.animation as animation

def draw_GPS_n_PC():

  file_path = "/home/anny/109-2/0618_mapExperiment/oneTree_MD/np_"

  pointX_world = []
  pointY_world = []
  egoX_world = np.array([])
  egoY_world = np.array([])

  for i in range(28):#29
      for j in range(0,10,2):
          pX, pY, egoX, egoY = np.load(file_path+str(i)+"_"+str(j)+".npy", allow_pickle=True)
          pointX_world.append(pX)
          pointY_world.append(pY)
          egoX_world = np.append(egoX_world,egoX)
          egoY_world = np.append(egoY_world,egoY)

  colors = cm.rainbow(np.linspace(1, 0, np.asarray(egoX_world).shape[0]))
  fig, ax = plt.subplots(figsize=(2,3),dpi=100)
  plt.grid(True)
  base = plt.gca().transData
  rotation = transforms.Affine2D().rotate_deg(92)#92
  for i in range(76,91):# len(egoX_world)
    plt.scatter(pointX_world[i],pointY_world[i], c=colors[i], transform = rotation + base, s=1)
    plt.scatter(egoX_world[i], egoY_world[i], c='k', transform = rotation + base, s=5)

  plt.xlim((-2.5, 1))
  plt.ylim((4, 12))
  plt.title('temporal PC result at 2m',fontsize=10)
  # plt.savefig(file_path+"2m.png", dpi=100)

  plt.show()

if __name__=='__main__':
  draw_GPS_n_PC()


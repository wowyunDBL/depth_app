#!usr/bin/env python
import csv
import numpy as np

def write2CSV:
    with open(file_path + 'tree.csv', 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(image)

def write2Numpy:
    np.save(file_path + 'two_tree', image)

def read4CSV:
    with open(file_path + 'far_1_p.csv', 'r') as csvfile:
        rows = csv.reader(csvfile, delimiter=',')
        for row in rows:
            npDepth.append(row)
    npDepth = np.asanyarray(npDepth)
    print(npDepth.dtype) # U5 means string length < 5
    print(npDepth.shape)
    npDepth = npDepth.astype('uint16')

def array_broadcasting:
    a = np.array([[1,2,3,4,5],[1,2,2,2,5],[4,4,3,2,5]])
    a[np.logical_and(a>3, a!=5)]=0
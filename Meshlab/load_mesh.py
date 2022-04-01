import numpy as np
import os, sys

def read_off(filename):
    num_select = 1024
    f = open(filename)
    f.readline()  # ignore the 'OFF' at the first line
    f.readline()  # ignore the second line
    All_points = []
    selected_points = []
    while True:
        new_line = f.readline()
        x = new_line.split(' ')
        if x[0] != '3':
            A = np.array(x[0:3], dtype='float32')
            All_points.append(A)
        else:
            break
    # if the numbers of points are less than 2000, extent the point set
    if len(All_points) < (num_select + 3):
        return None
    # take and shuffle points
    index = np.random.choice(len(All_points), num_select, replace=False)
    for i in range(len(index)):
        selected_points.append(All_points[index[i]])
    return selected_points  # return N*3 array

if __name__=='__main__':
    path = sys.argv[1]
    mesh = read_off(path)
    print("length of mesh is ", len(mesh))

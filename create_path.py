#!/usr/bin/env python
# coding: UTF-8

from __future__ import print_function

import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from PIL import Image
import numpy as np

class CreatePath:
    def __init__(self):
        self.counter = 0
        self.passing_size = 6 #通過点数
        self.passing_points = np.zeros([self.passing_size, 2])
        self.way_size = 50 #pathの点数
        self.way_points = np.zeros([self.way_size, 2])
        self.resolution = 0.05 #meter per pixel

    def spline(self, x,y,point,deg):
        tck,u = interpolate.splprep([x,y],k=deg,s=0) 
        u = np.linspace(0,1,num=point,endpoint=True) 
        spline = interpolate.splev(u,tck)
        return spline[0],spline[1]
    
    def motion(self, event):
        if event.button == 1:
            x = event.xdata
            y = event.ydata
            
            if self.counter < self.passing_size:
                plt.plot(x, y, marker='o')
                self.passing_points[self.counter, 0] = x
                self.passing_points[self.counter, 1] = y
            elif self.counter == self.passing_size:
                temp = self.spline(self.passing_points[:, 0], self.passing_points[:, 1],self.way_size,2)
                self.way_points[:, 0] = temp[0][:].T
                self.way_points[:, 1] = temp[1][:].T
                plt.plot(self.passing_points[:, 0], self.passing_points[:, 1], 'ro')
                plt.plot(self.way_points[:, 0],self.way_points[:, 1])
                with open('./path/path.csv', 'w') as f:
                    temp = self.way_points*self.resolution
                    writer = csv.writer(f)
                    writer.writerows(temp.tolist())

            self.counter += 1
        plt.draw()


if __name__ == '__main__':
    mycp = CreatePath()

    im = Image.open("./map/map.png")
    im_list = np.asarray(im)
    plt.imshow(im_list)
    plt.grid()
    plt.connect('button_press_event', mycp.motion)
    
    plt.show()
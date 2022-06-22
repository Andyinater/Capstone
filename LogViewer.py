# -*- coding: utf-8 -*-
"""
Created on Tue Jun 21 18:47:37 2022

@author: angwo
"""




# 78.5 diameter, outside tire to outside tire inches at full lock
#  %matplotlib qt

import csv
import matplotlib.pyplot as plt
import pylab

fname = "1655854509.4093537.txt"

f = open(fname, 'r')

cf = csv.reader(f, delimiter = '\t')

d = []

for i in cf:
    if float(i[7]) > 1.646151128502146e-38:
        d.append(i)

f.close()

steer = []
throttle = []
t = []
r = []
x = []
y = []

for i in d:
    steer.append(float(i[0]))
    throttle.append(float(i[1]))
    t.append(float(i[7]))
    r.append(float(i[8]))
    x.append(float(i[9]))
    y.append(float(i[10]))
    

fig,ax1 = plt.subplots()
fig.set_size_inches(16,16)
p1, = ax1.plot(throttle, color='red')
ax2 = ax1.twinx()
p2, = ax2.plot(r, color='blue')
ax3 = ax1.twinx()
p3, = ax2.plot(y, color='blue')
ax1.legend((p1,p2,p3),('throttle','r','x'))
fig.tight_layout()
plt.show()
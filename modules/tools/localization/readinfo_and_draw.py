#!/usr/bin/python
# -*- coding: latin-1 -*-
import matplotlib.pyplot as plt
import numpy as np
import re

clock = 0
time = []
stationerror= []
lateralerror= []
headingerror= []

input = open('/apollo/data/log/localization.INFO', 'r')
data=input.readlines()

for line in data:
     if re.findall('position error',line):
        line = line.split(',')
        station = line[5].split('[')[1].split(']')[0]
        lateral = line[6].split('[')[1].split(']')[0]
        stationerror.append((float)(station))
        lateralerror.append((float)(lateral))
        time.append(clock)
        clock+=0.1

for line1 in data:
     if re.findall('heading error',line1):
        line1 = line1.split()
        heading = line1[-1].split('[')[1].split(']')[0]
        headingerror.append(57.3*(float)(heading))

plt.title('Error Analysis')
plt.plot(time,stationerror,color='blue', label='$station$')
plt.plot(time,lateralerror,color='red',label='$lateral$')
plt.plot(time,headingerror,color='black',label='$heading$')
plt.legend()

#设置坐标轴名称
plt.xlabel('time(s)')
plt.ylabel('error(m)')
#plt.show()

#画gps图
trueX = []
trueY = []
for line2 in data:
     if re.findall('True position',line2):
        line2 = line2.split(',')
        x = line2[1].split('[')[1].split(']')[0]
        y = line2[2].split('[')[1].split(']')[0]
        trueX.append((float)(x))
        trueY.append((float)(y))

#print(trueX)
plt.figure('GPSX')
plt.plot(time,trueX, '.r')
plt.plot(time,trueX, 'b')

plt.figure('GPSY')
plt.plot(time,trueY, '.b',)
plt.plot(time,trueY,'r')

plt.show()

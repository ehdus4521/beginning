#!/usr/bin/env python

import rospy, roslaunch
import numpy as np
import subprocess
import os
import sys
import rospkg
import math
from enum import Enum
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ByteMultiArray, Int32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point , PoseStamped

'''with open("backup.txt",'r') as file:
            a=file.readlines()
            data=a[0].split(',')
            #print(data[0], data[1],data[2])

for i in range (1,11):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    data[2] = int(data[2])
    print(data[2])
    print("parking")

if readlines == '*':
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    data[2] = int(data[2])
    print(data[2])
    print("traffic")'''

f = open("backup.txt",'r')
line = f.readline()
print(line.count('*'))
lines = line.rsplit()
count = 0
for a in lines:
    if a == '*':
        count += 1
    else:
        pass
print('*:',count,'counting')
s_count = 0
for b in range(0,len(lines)):
    for c in range(0,len(lines[b])):
        if lines[b][c]=='*':
            s_count+=1
        else:
            continue
print(s_count)

'''story = open('backup.txt')
words = (word for line in story for word in line.split())

counts = dict()
for word in words:
    counts[word] = counts.get(word,0) + 1

tmp = [(v , k) for (k),(v) in counts.items()]
print(tmp)'''

'''for i in range (6,35):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("parking")

for i in range (84,89):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("safety_zone")

for i in range (89,92):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("crosswalk")

for i in range (93,107):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("safety_zone")

for i in range (108,114):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (125):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("crosswalk_timer")

for i in range (126,138):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (180,185):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (200,203):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("safety_zone")

for i in range (204,205):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("crosswalk")

for i in range (215,226):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (252,258):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (372,380):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")

for i in range (391,398):
    data=a[i].split(',')
    data[0] = float(data[0])
    data[1] = float(data[1])
    print(data[0], data[1])
    print("traffic")'''
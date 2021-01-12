#! /usr/bin/env python

import rospy
import time

def sum(data1,data2):

    sum=data1+data2

    return sum

def multi(data1,data2):

    multi=data1*data2

    return multi

if __name__ == '__main__':

    a=3
    b=5
    c=0
    prev_time=time.time()

    while not rospy.is_shutdown():

        if (time.time()-prev_time>=1):
            c+=1
            prev_time=time.time()

        result=sum(a,b)

        multi_result=multi(result,c)

        print(multi_result)

        if multi_result>80:
            c=0

    else:
        pass
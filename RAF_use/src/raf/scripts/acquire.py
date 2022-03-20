import rospy 
import math
import numpy as np
import cv2
import os

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


xlist = []
ylist = []
z = {}
img = 0
#订阅里程计
def odom_sub(data):
    xlist.append(data.pose.pose.position.x)
    ylist.append(data.pose.pose.position.y)
    #print(xlist[-1])
    #print(ylist[-1])
    #if math.sqrt((xlist[-1]-xlist[0])**2+(ylist[-1]-ylist[0])**2) < 0.5 and len(xlist) > 200:
    z = {'x':xlist,'y':ylist}
    print(z)
    
    #cv2.imshow('test',img)
    #cv2.waitKey(1)

#订阅摄像头
def img_sub(data):
    global img
    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)



if __name__=='__main__':
    rospy.init_node('acquire')


    rospy.Subscriber('/odom',Odometry,odom_sub)
    rospy.Subscriber("/camera/rgb/image_raw", Image, img_sub)

    while not rospy.is_shutdown():
        rospy.sleep(0.18)


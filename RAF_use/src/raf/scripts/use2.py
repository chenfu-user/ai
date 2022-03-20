#coding = utf-8
from dis import dis
import json
from re import S, T
import re
import rospy 
import math
import numpy as np
import cv2
import os


from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

def Reverse(lst):
    return [ele for ele in reversed(lst)]


hxlist = [-1.15,-1.48,1.99]
hylist = [-2.55,3.93,0.52]
lxlist = [-1.30,1.82,3.38]
lylist = [-1.44,4.14,-1.31]

jsonfile = open('b.json','r')

dlist = []

bflag = 0
res = json.load(jsonfile)

jsonfile.close()

#res['x'] = Reverse(res['x'])
#res['y'] = Reverse(res['y'])

def dist(x,y):
    return math.sqrt(x**2+y**2)

class PID():
    def __init__(self):
        self.kp = 3.8
        self.ki = 0.0
        self.kd = 0.6
        self.now = 0
        self.last = 0
        self.incont = 0
        self.dcont = 0

    def con(self,yaw,theta):
        f = theta - yaw
        leng = math.sin(f)
        print('leng:',leng)
        self.last = self.now
        self.now = leng
        self.incont += self.now
        if self.incont > 10:
            self.incont = 10
        elif self.incont < -10:
            self.incont = -10
        self.dcont   = self.now - self.last
        return self.now*self.kp + self.dcont*self.kd + self.incont*self.ki
        

class Trace:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.img = 0
        self.high_speed = 0.90
        self.low_speed = 0.60
        self.speed_flag = 0
        self.img_flag = 4758

        self.labelflag = 0

        self.vel = Twist()
        self.vel.linear.x = self.high_speed
        self.pid = PID()
        self.yaw = 0
        rospy.Subscriber('/odom',Odometry,self.odom_sub)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_sub)
        rospy.Subscriber("/imu",Imu,self.imu_sub)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

    def  odom_sub(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        self.labelflag += 1
        if self.labelflag == 2:     
            cv2.imwrite('img/'+str(self.img_flag)+'.jpg',self.img)
            l = {'x':self.vel.linear.x,'z':self.vel.angular.z}
            jsonfile = open('label/'+str(self.img_flag)+'.json','w')
            json.dump(l,jsonfile,indent=2)
            jsonfile.close()
            self.img_flag += 1
            self.labelflag = 0
            print('imgflag:',self.img_flag)

        cv2.imshow('test',self.img)
        cv2.waitKey(1)


        self.control()

    def imu_sub(self,data):
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        [row,pitch,yaw] = euler_from_quaternion([x,y,z,w])
        self.yaw = yaw



    def img_sub(self,data):
        self.img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        #img_gray = cv2.cvtColor(self.img, cv2.COLOR_RGB2GRAY)
        #_, dst = cv2.threshold(img_gray, 206, 255, cv2.THRESH_BINARY_INV)

        
        
        
    


    def control(self):
        global bflag
        dist_list = list(dist(self.x - res['x'][i], self.y - res['y'][i]) for i in range(len(res['x'])))
        #print(dist_list)
        length = len(dist_list)
        flag = 0
        
        min = dist_list[0]
        for i in range(bflag,bflag+60):
            # if i in dlist:
            #     continue
            if len(dist_list) -bflag <= 60:
                os.system('rosnode kill -a')

            if min>dist_list[i]:
                min = dist_list[i]
                flag = i

        print('flag:',flag)
        print('trace:',res['x'][flag],res['y'][flag])
        bflag = flag
        flag += 45
        if flag > length:
            flag -= length

        theta = math.atan2(res['y'][flag]-self.y , res['x'][flag]-self.x)
        self.vel.angular.z = self.pid.con(self.yaw, theta)



        for i in range(len(hxlist)):
            if dist(self.x - hxlist[i],self.y-hylist[i]) < 0.60:
                self.vel.linear.x = self.high_speed
                print(hxlist[i],hylist[i])
                print(self.x,self.y)
                break
            if dist(self.x - lxlist[i],self.y-lylist[i]) < 0.60:
                self.vel.linear.x = self.low_speed
                print(lxlist[i],lylist[i])
                print(self.x,self.y)
                break
        print('speed:',self.vel.linear.x)

        

        self.vel_pub.publish(self.vel)


if __name__=='__main__':
    rospy.init_node('trace_node')
    trace = Trace()

    while not rospy.is_shutdown():
        rospy.spin()

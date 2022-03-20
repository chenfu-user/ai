from raf import node
import time
import numpy as np

import rospy,os
import json
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

rospy.init_node("RAF")

myfile=open('a.txt','w')
odom_flag = 0
x = []
y = []
a = {'x':x,'y':y}


@node.speed_in.set_reader
def speed_in_reader(data):
    twist = Twist()

    twist.linear.x = data['linear_x']
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = data['angular_z']

    pub.publish(twist)


def callback(img_data):
    img = np.frombuffer(img_data.data, dtype=np.uint8).reshape(img_data.height, img_data.width, -1)
    node.img_out.write({'pic': img, 'time': time.time()})

def odom_sub(data):
    global odom_flag,x,y,a
    #print(odom_flag)
    if odom_flag < 2:
        x.append(data.pose.pose.position.x)
        y.append(data.pose.pose.position.y)
    if odom_flag == 0  :
        if math.sqrt((x[-1]-x[0])**2+(y[-1]-y[0])**2) < 0.5:
            odom_flag = 0
        else :
            odom_flag = 1
    if odom_flag == 1 :
        if math.sqrt((x[-1]-x[0])**2+(y[-1]-y[0])**2) < 0.5:
            odom_flag = 2

    if odom_flag == 2:
        a = {'x':x,'y':y}
        json.dump(a,myfile,indent=2)
        myfile.close()
        print(a)
        odom_flag = 3
        os.system('rosnode kill -a')

    


odom = rospy.Subscriber('/odom',Odometry,odom_sub)
sub = rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)



# rospy.spin()


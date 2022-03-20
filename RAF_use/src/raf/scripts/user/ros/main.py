from raf import node
import time
import numpy as np

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

rospy.init_node("RAF")


@node.speed_in.set_reader
def speed_in_reader(data):
    twist = Twist()

    twist.linear.x = data['linear_x']
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = data['angular_z']
    print(twist)
    pub.publish(twist)


def callback(img_data):
    img = np.frombuffer(img_data.data, dtype=np.uint8).reshape(img_data.height, img_data.width, -1)
    node.img_out.write({'pic': img, 'time': time.time()})


sub = rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

# rospy.spin()


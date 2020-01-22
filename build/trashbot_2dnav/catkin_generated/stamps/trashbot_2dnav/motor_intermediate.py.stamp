#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

LINEAR_VEL_SCALE=1.0
TWIST_VEL_SCALE=1.0
cmd_vel_pub = rospy.Publisher('/cmd_vel', String, queue_size=10)
rospy.init_node('motor_intermediate', anonymous=True)

def callback(data):
    new_vel = Twist()
    new_vel.linear = data.linear
    new_vel.angular = data.angular
    cmd_vel_pub.publish(new_vel)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber("/inter_cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
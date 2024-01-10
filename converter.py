#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from speed.msg import val_encoder

SCALE_FACTOR_VEL_LEFT =  293.825
SCALE_FACTOR_VEL_RIGHT =  293.825
# rosrun rqt_plot rqt_plot
L = 1 #scale khoang cach 2 banh xe
SCALE_FACTOR_ANG_LEFT  =  369.231*L/2
SCALE_FACTOR_ANG_RIGHT  = 369.231*L/2
#What do you want to do
#Read pose from /turtle1/pos
#archieve /turtle1/cmd_vel
encoder_pub = rospy.Publisher('/encod_val',val_encoder,queue_size=10)
data = val_encoder()
#define Subscriber
def callback(vel_sub_msg):
    global val_encod_right
    global val_encod_left
    # val_encod_left = 0
    # val_encod_right = 0
    if vel_sub_msg.angular.z == 0:
        val_encod_left = vel_sub_msg.linear.x*SCALE_FACTOR_VEL_LEFT
        val_encod_right = vel_sub_msg.linear.x*SCALE_FACTOR_VEL_RIGHT
        rospy.loginfo("haha")
    else:
        if vel_sub_msg.angular.z > 0:
            rospy.loginfo("hehe")
            val_encod_left = -vel_sub_msg.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = vel_sub_msg.angular.z*SCALE_FACTOR_ANG_RIGHT
        else:
            val_encod_left = -vel_sub_msg.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = vel_sub_msg.angular.z*SCALE_FACTOR_ANG_RIGHT
    data.encoder_left = int(val_encod_left)
    data.encoder_right = int(val_encod_right)
    encoder_pub.publish(data)
    # rospy.loginfo(type(vel_sub_msg.angular.z))
    # rospy.loginfo(val_encod_right+1)
    


#define Publisher
def subpub():
    global val_encod_right
    global val_encod_left
    rospy.init_node('converter',anonymous=True)
    rospy.Subscriber('/cmd_vel',Twist,callback)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        rate.sleep()
        encoder_pub.publish(data)
    # rospy.spin()

if __name__ == '__main__':
    try:
        subpub()
    except rospy.ROSInterruptException:
        pass

        


     
#!/usr/bin/env python3
import rospy
import serial
from time import sleep
from std_msgs.msg import Int32,Float32,String
from geometry_msgs.msg import Twist

SCALE_FACTOR_VEL_LEFT = 2.2*3.14159*0.065
SCALE_FACTOR_VEL_RIGHT =  2.2*3.14159*0.065
# rosrun rqt_plot rqt_plot
L = 0.195 #scale khoang cach 2 banh xe la 19.5cm
D = 0.65 #duong kinh banh xe la 65mm
PHI = L*0.5
SCALE_FACTOR_ANG_LEFT  =  100
SCALE_FACTOR_ANG_RIGHT  = 100
v_L = 0
v_R = 0 
vel_pub_msg = Twist()
setpoint_left = 0
setpoint_right = 0
encoder = Twist()
encoder_rx = String()
encoder1_right_temp = String()
encoder2_left_temp = String()
setpoint1_temp = String()
setpoint2_temp = String()
setpoint = Twist()
encoder1_right = 0
encoder2_left = 0	

def MOBILE_Vel_Cal():
    global v_L, v_R, encoder2_left, encoder1_right
    global vel_pub_msg
    v_L = SCALE_FACTOR_VEL_LEFT*encoder2_left 
    v_R = SCALE_FACTOR_VEL_RIGHT*encoder1_right
    vel_pub_msg.linear.x = (v_L + v_R)/2.0/125
    vel_pub_msg.angular.z = (v_R - v_L)/L/125
	
def main(): 
    global encoder1_right, encoder2_left
    rospy.init_node('read_encoder', anonymous=True)
#    rat = rospy.Rate(100)
    ser = serial.Serial(
	port = '/dev/ttyUSB1',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1) 
    setpoint_ = rospy.Publisher('/setpoint12', Twist, queue_size=10)
    encoder_pub = rospy.Publisher('/encoder', Twist, queue_size=10)
    vel_pub = rospy.Publisher('vel_pub', Twist, queue_size=10)
    while not rospy.is_shutdown():
    	try:
	    	while True:
	    		ser.flush()
	    		encoder_rx = ser.readline().decode('utf-8')
	    		#rospy.logwarn(encoder_rx)
	    		if len(encoder_rx) == 24: 
		    		ser.flush()
		    		encoder1_right_temp = encoder_rx[:7]
		    		encoder2_left_temp = encoder_rx[8:15]
		    		setpoint1_temp = encoder_rx[15:18]
		    		setpoint2_temp = encoder_rx[19:22]
		    		if setpoint1_temp[0] == '+':
		    			setpoint.linear.x = float(setpoint1_temp[1:])
		    		elif setpoint1_temp[0] == '-':
		    			setpoint.linear.x = -float(setpoint1_temp[1:])
		    		if setpoint2_temp[0] == '+':
		    			setpoint.linear.y = float(setpoint2_temp[1:])
		    		elif setpoint2_temp[0] == '-':
		    			setpoint.linear.y = -float(setpoint2_temp[1:])
		    		if encoder1_right_temp[0] == '+':
		    			encoder1_right = float(encoder1_right_temp[2:7])
		    		elif encoder1_right_temp[0] == '-':
		    			encoder1_right = -float(encoder1_right_temp[2:7])
		    		else:
		    			rospy.logwarn("OKOK")
		    		if encoder2_left_temp[0] == '+':
		    			encoder2_left = float(encoder2_left_temp[2:7])
		    		elif encoder2_left_temp[0] == '-':
		    			encoder2_left = -float(encoder2_left_temp[2:7])
		    		#rospy.loginfo(encoder_rx)
		    		encoder.linear.x = encoder1_right
		    		encoder.linear.y = encoder2_left
		    		encoder_pub.publish(encoder)
		    		MOBILE_Vel_Cal()
		    		vel_pub.publish(vel_pub_msg)
                               
		    	else:
		    		rospy.logerr("No data")
		    		#break
		    		
    	except KeyboardInterrupt:
    		ser.close()
    		rospy.logwarn("ERR")
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

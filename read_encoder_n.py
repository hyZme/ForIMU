#!/usr/bin/env python3

import rospy
import serial
from time import sleep
from std_msgs.msg import Int32,Float32,String
from geometry_msgs.msg import Twist
#from encoder.msg import val_encoder

SCALE_FACTOR_VEL_LEFT =  293.825
SCALE_FACTOR_VEL_RIGHT =  293.825
# rosrun rqt_plot rqt_plot
L = 1 #scale khoang cach 2 banh xe
SCALE_FACTOR_ANG_LEFT  =  100
SCALE_FACTOR_ANG_RIGHT  = 100

tx_buffer = '+00.00/+00.00/y/n'

def Callback1(value):
    global a
    b=a
    a=a+1
    
    global setpoint_left
    global setpoint_right
    setpoint = Twist()
    ser = serial.Serial(
	port = '/dev/ttyUSB1',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1)
    tx_buffer = '+00.00/+00.00/y/n'
	
    if value.angular.z == 0:
        val_encod_left = value.linear.x*SCALE_FACTOR_VEL_LEFT
        val_encod_right = value.linear.x*SCALE_FACTOR_VEL_RIGHT
    else:
        if value.angular.z > 0:
            val_encod_left = 0 #-value.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = value.angular.z*SCALE_FACTOR_ANG_RIGHT
        else:
            val_encod_left = value.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = 0 #value.angular.z*SCALE_FACTOR_ANG_RIGHT
    setpoint_ = rospy.Publisher('/setpoint12', Twist, queue_size=10)
    setpoint.linear.x = val_encod_left
    setpoint.linear.y = val_encod_right
    while a-b == 1:
    	try:
    		setpoint_.publish(setpoint)
    	except: 
    		break
    vel_left = round(val_encod_left)
    vel_right = round(val_encod_right)
    #rospy.loginfo("OK")
    
    if vel_left and vel_right > 0:
        tx_buffer = '+' + str(vel_left) +'.00/+' + str(vel_right) + '.00/y/n'
        rospy.loginfo("Forward")
    elif vel_left == 0 and vel_right > 0:
    	tx_buffer = '+00.00/+' + str(vel_right) + '.00/y/n'
    	rospy.loginfo("Turn Left")
    elif vel_left > 0 and vel_right == 0:
    	tx_buffer = '+'+str(vel_left) +'.00/+00.00/y/n'
    	rospy.loginfo("Turn Right")
    elif vel_left < 0 and vel_right < 0:
    	tx_buffer = str(vel_left) +'.00/' + str(vel_right) + '.00/y/n'
    	rospy.loginfo("Backward")
    ser.write(tx_buffer.encode())
    rospy.logwarn(tx_buffer)
    
    #encoder_pub.publish(encoder)
def main(): 
    a= 0
    global val_encod_left
    global setpoint_
    global setpoint_right
    #port_name = rospy.get_param('~port','/dev/ttyUSB1')
    #baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(
	port = '/dev/ttyUSB1',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1) 
    rospy.init_node('read_encoder', anonymous=True)
    rat = rospy.Rate(100)
    encoder1_left_pub = rospy.Publisher('/encod1_left_val', Float32, queue_size=10)
    encoder2_right_pub = rospy.Publisher('/encod2_right_val', Float32, queue_size=10)
    setpoint_ = rospy.Publisher('/setpoint12', Twist, queue_size=10)
    #setpoint_right = rospy.Publisher('/setpoint_right', Float32, queue_size=10)
    rospy.Subscriber('/cmd_vel',Twist, Callback1)
    encoder = Float32()
 #   while True:
  #  	if ser.in_waiting > 0:
   # 	 encoder_rx = ser.readline().decode('utf-8').rstrip() 
    #	 encoder_temp = encoder_rx[:7]
    #	 rospy.loginfo(encoder_rx)
    #	 rospy.logwarn(encoder)
    #	else:
    #	 rospy.logwarn("KO")

		
    while not rospy.is_shutdown():
    	rat.sleep()
    	#encoder_pub.publish(encoder)
    	try:
	    	while True:
	    		#setpoint_left.publish(val_encod_left)
	    		encoder_rx = ser.readline().decode('utf-8')
	    		ser.flush()
	    		encoder1_left_temp = encoder_rx[:7]
	    		encoder2_right_temp = encoder_rx[8:15]
	    		
	    		if encoder1_left_temp[0] == '+':
	    			encoder1_left = float(encoder1_left_temp[2:7])
	    		elif encoder1_left_temp[0] == '-':
	    			encoder1_left = -float(encoder1_left_temp[2:7])
	    		else:
	    			rospy.logwarn("OKOK")
	    		if encoder2_right_temp[0] == '+':
	    			encoder2_right = float(encoder2_right_temp[2:7])
	    		elif encoder2_right_temp[0] == '-':
	    			encoder2_right = -float(encoder2_right_temp[2:7])
	    		#rospy.loginfo(encoder_rx)
	    		#rospy.logwarn(encoder1_left_temp)
	    		#rospy.loginfo("------")
	    		#rospy.logwarn(encoder2_right_temp)
	    		encoder1_left_pub.publish(encoder1_left)
	    		encoder2_right_pub.publish(encoder2_right) 
    	except KeyboardInterrupt:
    		ser.close()
    		break
    	
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    #rospy.logwarn(type(tx_buffer))

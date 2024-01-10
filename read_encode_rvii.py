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
SCALE_FACTOR_ANG_LEFT  =  100*L/2
SCALE_FACTOR_ANG_RIGHT  = 100*L/2

tx_buffer = '+00.00+00.00/y/n'
def Callback1(value):
    ser = serial.Serial(
	port = '/dev/ttyUSB0',
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
            val_encod_left = -value.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = value.angular.z*SCALE_FACTOR_ANG_RIGHT
        else:
            val_encod_left = -value.angular.z*SCALE_FACTOR_ANG_LEFT
            val_encod_right = value.angular.z*SCALE_FACTOR_ANG_RIGHT
    vel_left = val_encod_left
    vel_right = val_encod_right
    rospy.loginfo("OK")
    
    rospy.logwarn(type(tx_buffer))
    if vel_left and vel_right >= 0:
        tx_buffer = '+' + str(vel_left) +'.00/+' + str(vel_right) + '.00/y/n'
    elif vel_left >=0 and vel_right < 0:
    	tx_buffer = '+' + str(vel_left) +'.00/-' + str(vel_right) + '.00/y/n'
    elif vel_left < 0 and vel_right >= 0:
    	tx_buffer = '-' + str(vel_left) +'.00/+' + str(vel_right) + '.00/y/n'
    elif vel_left < 0 and vel_right < 0:
    	tx_buffer = '-' + str(vel_left) +'.00/-' + str(vel_right) + '.00/y/n'
    ser.write(tx_buffer.encode())
    rospy.logwarn(tx_buffer)
def main():
    #port_name = rospy.get_param('~port','/dev/ttyUSB1')
    #baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1) 
    rospy.init_node('read_encoder', anonymous=True)
    rat = rospy.Rate(100)
    encoder_pub = rospy.Publisher('/encod_val', Float32, queue_size=10)
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
    	try:
	    	while True:
	    		encoder_rx = ser.readline().decode('utf-8')
	    		ser.flush()
	    		encoder_temp = encoder_rx[:7]
	    		rospy.loginfo(encoder_rx)
	    		rospy.logwarn(encoder_temp)
	    		if encoder_temp[0] == '+':
	    			encoder = float(encoder_temp[2:7])
	    		elif encoder_temp[0] == '-':
	    			encoder = -float(encoder_temp[2:7])
	    		else:
	    			rospy.logwarn("OKOK")
    	except KeyboardInterrupt:
    		ser.close()
    		break
    	encoder_pub.publish(encoder)
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

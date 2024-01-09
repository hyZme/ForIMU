#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32,Float32,String
from geometry_msgs.msg import Twist

tx_buffer = '+00.00/+00.00/y/n'
def Callback1(value):
    ser = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 
    tx_buffer = '+00.00/+00.00/y/n'
    vel = value.data
    rospy.loginfo("OK")
    rospy.loginfo(type(vel))
    rospy.logwarn(type(tx_buffer))
    if vel >= 0:
        tx_buffer = '+' + str(int(vel/10)) + str(int(vel%10)) + tx_buffer[3:]
    else:
    	tx_buffer = '-' + str(int(vel/10)) + str(int(vel%10)) + tx_buffer[3:]
    ser.write(tx_buffer.encode())
    rospy.logwarn(tx_buffer)
def main():
    #port_name = rospy.get_param('~port','/dev/ttyUSB1')
    #baud = int(rospy.get_param('~baud','115200'))
    ser = serial.Serial(port = '/dev/ttyUSB0', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS, timeout=1) 
    rospy.init_node('read_encoder', anonymous=True)
    rat = rospy.Rate(100)
    encoder_pub = rospy.Publisher('/encod_val', Float32, queue_size=10)
    rospy.Subscriber('/cmd_vel',Int32, Callback1)
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
    	while True:
	    	if ser.in_waiting > 0:
		    	encoder_rx = ser.readline().decode('utf-8').rstrip() 
		    	encoder_temp = encoder_rx[:7]
		    	rospy.loginfo(encoder_rx)
		    	rospy.logwarn(encoder_temp)
		    	if encoder_temp[0] == '+':
		    		encoder = float(encoder_temp[2:7])
		    	elif encoder_temp[0] == '-':
		    		encoder = -float(encoder_temp[2:7])
		    	else:
		    		rospy.logwarn("OKOK")
		    	encoder_pub.publish(encoder)
	
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

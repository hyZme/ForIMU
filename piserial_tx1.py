#!/usr/bin/env python3
import rospy
import serial
from time import sleep
from std_msgs.msg import Int32,Float32,String
from geometry_msgs.msg import Twist

SCALE_FACTOR_VEL_LEFT =  3.14159*0.65
SCALE_FACTOR_VEL_RIGHT =  3.14159*0.65
# rosrun rqt_plot rqt_plot
L = 0.195 #scale khoang cach 2 banh xe la 19.5cm
D = 0.65 #duong kinh banh xe la 65mm
PHI = L*0.5
SCALE_FACTOR_ANG_LEFT  =  100
SCALE_FACTOR_ANG_RIGHT  = 100
tx_buffer = '+00.00/+00.00/y/n'
linear_x = 0.0
angular_z = 0.0
v_L = 0.0
v_R = 0.0
encoder_left = 0
encoder_right = 0
vel_pub_msg = Twist()
setpoint_left = 0
setpoint_right = 0
vel_left = 0 #toc do encoder banh trai sau khi lam tron
vel_right = 0
encoder = Twist()
encoder_rx = String()
encoder1_right_temp = String()
encoder2_left_temp = String()
setpoint1_temp = String()
setpoint2_temp = String()
setpoint = Twist()
encoder1_right = Float32()
encoder2_left = Float32()	
flag = 0x0
flag_tx = 0x0
vel_left_tx = 0
vel_right_tx = 0
def MOTOR_Enc_Cal():
    global encoder_left, encoder_right, linear_x, angular_z
    v_L = linear_x - angular_z * PHI*25
    v_R = linear_x + angular_z * PHI*25
    encoder_left = v_L/SCALE_FACTOR_VEL_LEFT
    encoder_right = v_R/SCALE_FACTOR_VEL_RIGHT
    #rospy.loginfo(v_L)
    #rospy.loginfo(v_R)
def Callback1(value):
    global encoder_left, encoder_right, linear_x, angular_z
    global setpoint_left,vel_left,vel_right
    global setpoint_right, flag
    global tx_buffer
    global flag_tx
    #tx_buffer = '+00.00/+00.00/y/n'
    linear_x = value.linear.x
    angular_z = value.angular.z
    v_L = linear_x - angular_z * PHI
    v_R = linear_x + angular_z * PHI
    encoder_left = v_L/SCALE_FACTOR_VEL_LEFT*590.0
    encoder_right = v_R/SCALE_FACTOR_VEL_RIGHT*590.0
   # rospy.logwarn(encoder_left)
   # rospy.logwarn(encoder_right)
    vel_left = round(encoder_left)
    vel_right = round(encoder_right)
    vel_left_tx = abs(vel_left)
    vel_right_tx = abs(vel_right)
    if(vel_left_tx < 10 and vel_left_tx > 1 and vel_right_tx < 10 and vel_right_tx > 1):
        flag_tx = 0x1
    elif (vel_left_tx < 10 and vel_left_tx > 1 and vel_right_tx > 10):
        flag_tx = 0x2
    elif(vel_right_tx < 10 and vel_right_tx > 1 and vel_left_tx > 10):
        flag_tx = 0x3    
    elif (vel_left_tx > 10 and vel_right_tx > 10):
        flag_tx = 0x4
    if(flag_tx == 0x1):
	    if(vel_left > 0 and vel_right > 0):
	       x_buffer = '+0' + str(vel_left) +'.00/+0' + str(vel_right) + '.00/y/n'
	    elif (vel_left < 0 and vel_right > 0):
	    	tx_buffer =  '+0' + str(vel_right) +'.00/-0' + str(abs(vel_left)) + '.00/y/n'
	    elif vel_left > 0 and vel_right < 0:
	    	tx_buffer = '-0'+str(abs(vel_right)) + '.00/+0'+ str(vel_left)+'.00/y/n'
	    elif vel_left < 0 and vel_right < 0:
	    	tx_buffer = '-0' + str(abs(vel_left))+'.00/-0' + str(abs(vel_right)) + '.00/y/n'
	    elif vel_left == 0 and vel_right == 0:
	    	tx_buffer = '+00.00/+00.00/y/n'
    elif(flag_tx == 0x2):
            if vel_left > 0 and vel_right > 0:
               tx_buffer = '+0' + str(vel_left) +'.00/+' + str(vel_right) + '.00/y/n'
            elif (vel_left < 0 and vel_right > 0):
               tx_buffer =  '+' + str(vel_right) +'.00/-0' + str(abs(vel_left)) + '.00/y/n'
            elif vel_left > 0 and vel_right < 0:
               tx_buffer = str(vel_right) + '.00/+0'+ str(vel_left)+'.00/y/n'
            elif vel_left < 0 and vel_right < 0:
               tx_buffer = '-0' + str(abs(vel_left))+'.00/' + str(vel_right) + '.00/y/n'
            elif vel_left == 0 and vel_right == 0:
               tx_buffer = '+00.00/+00.00/y/n'
    elif(flag_tx == 0x3):
            if vel_left > 0 and vel_right > 0:
               tx_buffer = '+' + str(vel_left) +'.00/+0' + str(vel_right) + '.00/y/n'
            elif vel_left < 0 and vel_right > 0:
               tx_buffer =  '+0' + str(vel_right) +'.00/' + str(vel_left) + '.00/y/n'
            elif vel_left > 0 and vel_right < 0:
               tx_buffer = '-0'+str(abs(vel_right)) + '.00/+'+ str(vel_left)+'.00/y/n'
            elif vel_left < 0 and vel_right < 0:
               tx_buffer =  str(vel_left)+'.00/-0' + str(abs(vel_right)) + '.00/y/n'
            elif vel_left == 0 and vel_right == 0:
               tx_buffer = '+00.00/+00.00/y/n'
    elif(flag_tx == 0x4):
            if vel_left > 0 and vel_right > 0:
               tx_buffer = '+' + str(vel_left) +'.00/+' + str(vel_right) + '.00/y/n'
            elif vel_left < 0 and vel_right > 0:
               tx_buffer =  '+' + str(vel_right) +'.00/' + str(vel_left) + '.00/y/n'
            elif vel_left > 0 and vel_right < 0:
               tx_buffer = str(vel_right) + '.00/+'+ str(vel_left)+'.00/y/n'
            elif vel_left < 0 and vel_right < 0:
               tx_buffer = str(vel_left)+'.00/' + str(vel_right) + '.00/y/n'
            elif vel_left == 0 and vel_right == 0:
               tx_buffer = '+00.00/+00.00/y/n'
    
    flag = 0x1;
def main(): 
    global flag,tx_buffer
    rospy.init_node('read_encoder', anonymous=True)
    rat = rospy.Rate(10)
    ser = serial.Serial(
	port = '/dev/ttyUSB1',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1) 

    rospy.Subscriber('/cmd_vel',Twist, Callback1)
    
    while not rospy.is_shutdown():
    	if flag == 0x1:
    	   flag = 0x0;
    	   ser.write(tx_buffer.encode())
    	   rospy.logwarn(tx_buffer)
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

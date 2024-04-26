#!/usr/bin/env python3

import math
import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_about_axis
from mpu9250_i2c import*
import numpy as np

ADDR1 = None
ADDR = None
bus = None
bus1 = None
IMU_FRAME = None
MAG_FRAME = None
PWR_MGMT_1   = 0x6B

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word1(adr1):
   # high = bus1.read_byte_data(ADDR1, adr1)
   #low = bus1.read_byte_data(ADDR1, adr1+1)
    high = bus1.read_byte_data(ADDR1, adr1)
    low = bus1.read_byte_data(ADDR1, adr1-1)
   # val = (high << 8) + low
    val = ((high << 8) | low)
   # if (val > 32768):
    #   value -= 65536
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def read_word_2c1(adr1):
    val = read_word1(adr1)
    if (val > 32768):
        val -= 65536
   # if (val >= 0x8000):
    #    return -((65535 - val) + 1)
   #else:
        return val
    
accel_x = 0
accel_y = 0
accel_z = 0

gyro_x = 0
gyro_y = 0
gyro_z = 0

def publish_imu():
    global accel_x,accel_y,accel_z
    global gyro_x,gyro_y,gyro_z

    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

    imu_msg.linear_acceleration.x = accel_x*9.81 
    imu_msg.linear_acceleration.y = accel_y*9.81 
    imu_msg.linear_acceleration.z = accel_z*9.81

    imu_msg.angular_velocity.x = gyro_x*0.0174 
    imu_msg.angular_velocity.y = gyro_y*0.0174 
    imu_msg.angular_velocity.z = gyro_z*0.0174

    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.orientation_covariance = [2.6e-07, 0.0,0.0,0.0, 2.6e-07, 0.0,0.0,0.0,0.0]
    imu_msg.angular_velocity_covariance = [2.5e-04,0.0,0.0,0.0,2.5e-04,0.0,0.0,0.0,2.5e-04]
    imu_msg.linear_acceleration_covariance = [2.5e-04,0.0,0.0,0.0,2.5e-04,0.0,0.0,0.0,2.5e-04]
    imu_pub.publish(imu_msg)

imu_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)
        
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)
    imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    rospy.spin()
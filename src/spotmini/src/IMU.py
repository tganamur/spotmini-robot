#!/usr/bin/env python3

import time

import numpy as np

import board
import busio
import adafruit_bno055

import rospy
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Header
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


def main():
    rospy.init_node("imu")

    port = rospy.get_param("~port", "RPI_1")
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    pub_imu = rospy.Publisher("~imu", Imu, queue_size=10)
    pub_magn = rospy.Publisher("~magnetometer", MagneticField, queue_size=10)

    msg_imu = Imu()
    msg_magn = MagneticField()
    hdr = Header(stamp=rospy.Time.now(), frame_id="IMU")

    rate = rospy.Rate(rospy.get_param('~hz', 10))
    while not rospy.is_shutdown():
        q = sensor.quaternion        # x,y,z,w
        mag = sensor.magnetic    # micro Tesla (µT)
        gyro = sensor.gyro      # deg/second
        accel = sensor.acceleration # m/s²
        euler = sensor.euler
        
        msg_imu.header = hdr

        hdr.stamp = rospy.Time.now()
            
        if q is not None and accel is not None and gyro is not None:
            msg_imu.header = hdr
            msg_imu.linear_acceleration.x = euler[0]
            msg_imu.linear_acceleration.y = euler[1]
            msg_imu.linear_acceleration.z = euler[2]
            msg_imu.angular_velocity.x = math.radians(gyro[0])
            msg_imu.angular_velocity.y = math.radians(gyro[1])
            msg_imu.angular_velocity.z = math.radians(gyro[2])
            msg_imu.orientation.w = q[3]
            msg_imu.orientation.x = q[0]
            msg_imu.orientation.y = q[1]
            msg_imu.orientation.z = q[2]
            try:
                pub_imu.publish(msg_imu)
            except:
                pass
            
        if mag is not None:
            msg_magn.header = hdr
            msg_magn.magnetic_field.x = mag[0]*1e-6
            msg_magn.magnetic_field.y = mag[1]*1e-6
            msg_magn.magnetic_field.z = mag[2]*1e-6
            try:
                pub_magn.publish(msg_magn)
            except:
                rospy.logwarn("Mag msg has incorrect datatype")

        rate.sleep()


if __name__ == '__main__':
    main()

    

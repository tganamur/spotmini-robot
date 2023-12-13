#!/usr/bin/env python3

import numpy as np
import time
import math

import rospy

from Controller import Controller
from State import State
from HardwareInterface import HardwareInterface
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics
from ServoCalibration import NEUTRAL_ANGLE_DEGREES, MIN, MAX
from ServoCalibration import NEUTRAL_ANGLE_DEGREES, MIN, MAX
from std_msgs.msg import Float64
from Command import Command
from std_msgs.msg import Float64
from JoystickInterface import JoystickInterface
from sensor_msgs.msg import Imu

config = Configuration()
hardware_interface = HardwareInterface()

last_roll = 0

# Create controller and user input handles
controller = Controller(
    config,
    four_legs_inverse_kinematics,
)
state = State()

def is_leaning_right():
    return hardware_interface.get_current_actuator_position()[0][0] > NEUTRAL_ANGLE_DEGREES[0][0] and hardware_interface.get_current_actuator_position()[0][2] > NEUTRAL_ANGLE_DEGREES[0][2]

def is_leaning_left():
    return hardware_interface.get_current_actuator_position()[0][1] > NEUTRAL_ANGLE_DEGREES[0][1] and hardware_interface.get_current_actuator_position()[0][3] > NEUTRAL_ANGLE_DEGREES[0][3]

def process_input(message):
    
    global config
    global hardware_interface
    global controller
    global state
    global last_roll

    linear_acc = message.linear_acceleration
    
    # roll = math.atan2(linear_acc.x, linear_acc.z) * 57.3
    roll = linear_acc.y
    rospy.loginfo("Roll: %s", str(roll))
    # rospy.loginfo("Currentpos: %s", str(np.shape(hardware_interface.get_current_actuator_position())))

    height_speed = 3.0

    leaning_right = np.array(
            [[+height_speed, 0., +height_speed, 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.]]
        )

    right_compensate = leaning_right * -1.

    leaning_left = np.array(
        [[0., -height_speed, 0., -height_speed],
        [0., 0., 0., 0.],
        [0., 0., 0., 0.]]
    )

    left_compensate = leaning_left * -1.

    new_pos = hardware_interface.get_current_actuator_position()
    
    if np.abs(roll) > 4:
        if (roll > 0):
            if roll - last_roll > 0:
                rospy.loginfo("Roll > 0, leaning right")
                new_pos = hardware_interface.get_current_actuator_position() + leaning_right
                new_pos = np.clip(new_pos, MIN, MAX)
            else:
                rospy.loginfo("Roll > 0, leaning left")
            # decrease left legs
            # increasing matrix
                new_pos = hardware_interface.get_current_actuator_position() + right_compensate
                new_pos = np.clip(new_pos, MIN, MAX)

        elif (roll < 0):
            if roll - last_roll < 0:
                rospy.loginfo("Roll < 0, leaning left")
                new_pos = hardware_interface.get_current_actuator_position() + leaning_left
                new_pos = np.clip(new_pos, MIN, MAX)
            #decreasing
            else:
                rospy.loginfo("Roll < 0, leaning right")
                new_pos = hardware_interface.get_current_actuator_position() + left_compensate
                new_pos = np.clip(new_pos, MIN, MAX)
    else:
        new_pos = NEUTRAL_ANGLE_DEGREES

    rospy.loginfo("Last roll: %s\nNew pos: %s", str(last_roll), str(new_pos))
    last_roll = roll

    # Update the pwm widths going to the servos
    hardware_interface.set_actuator_postions(new_pos)

def main():
    rospy.init_node("spotmini")

    # Create config
    global config
    global hardware_interface
    global controller
    global state
    
    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)
    
    hardware_interface.set_actuator_postions(NEUTRAL_ANGLE_DEGREES)
    
    imu_info = rospy.Subscriber("imu/imu", Imu, process_input)
    
    rospy.spin()


if __name__ == "__main__":
    main()

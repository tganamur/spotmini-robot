#!/usr/bin/env python3

from Config import ServoParams, PWMParams

from board import SCL, SDA
import busio

import numpy as np
from ServoCalibration import NEUTRAL_ANGLE_DEGREES

from adafruit_servokit import ServoKit

class HardwareInterface:
    def __init__(self):
        self.pca = ServoKit(channels=16)
        self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.current_actuator_positions = NEUTRAL_ANGLE_DEGREES
        self.pca.servo[2].set_pulse_width_range(1000, 2500)
        self.pca.servo[6].set_pulse_width_range(500, 1950)
        self.pca.servo[10].set_pulse_width_range(1000, 2500)
        self.pca.servo[14].set_pulse_width_range(500, 2000)

    def set_actuator_postions(self, joint_angles):
        send_servo_commands(self.pca, self.pwm_params, self.servo_params, joint_angles)
        self.current_actuator_positions = joint_angles
    
    def set_actuator_position(self, joint_angle, axis, leg):
        send_servo_command(self.pca, self.pwm_params, self.servo_params, joint_angle, axis, leg)
        
    def get_current_actuator_position(self):
        return self.current_actuator_positions


def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
    """Converts a pwm signal (measured in microseconds) to a corresponding duty cycle on the gpio pwm pin

    Parameters
    ----------
    pulsewidth_micros : float
        Width of the pwm signal in microseconds
    pwm_params : PWMParams
        PWMParams object

    Returns
    -------
    float
        PWM duty cycle corresponding to the pulse width
    """
    return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwm_params.range)


def angle_to_pwm(angle, servo_params, axis_index, leg_index):
    """Converts a desired servo angle into the corresponding PWM command

    Parameters
    ----------
    angle : float
        Desired servo angle, relative to the vertical (z) axis
    servo_params : ServoParams
        ServoParams object
    axis_index : int
        Specifies which joint of leg to control. 0 is abduction servo, 1 is inner hip servo, 2 is outer hip servo.
    leg_index : int
        Specifies which leg to control. 0 is front-right, 1 is front-left, 2 is back-right, 3 is back-left.

    Returns
    -------
    float
        PWM width in microseconds
    """
    angle_deviation = (
        angle - servo_params.neutral_angles[axis_index, leg_index]
    ) * servo_params.servo_multipliers[axis_index, leg_index]
    pulse_width_micros = (
        servo_params.neutral_position_pwm
        + servo_params.micros_per_rad * angle_deviation
    )
    return pulse_width_micros


def angle_to_duty_cycle(angle, pwm_params, servo_params, axis_index, leg_index):
    return pwm_to_duty_cycle(
        angle_to_pwm(angle, servo_params, axis_index, leg_index), pwm_params
    )


def initialize_pwm(pca, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pca.frequency = pwm_params.freq


def send_servo_commands(pca, pwm_params, servo_params, joint_angles):
    for leg_index in range(4):
        for axis_index in range(3):
            # duty_cycle = angle_to_duty_cycle(
            #     joint_angles[axis_index, leg_index],
            #     pwm_params,
            #     servo_params,
            #     axis_index,
            #     leg_index,
            # )
            pca.servo[pwm_params.pins[axis_index, leg_index]].angle = joint_angles[axis_index, leg_index]


def send_servo_command(pca, pwm_params, servo_params, joint_angle, axis, leg):
    # duty_cycle = angle_to_duty_cycle(joint_angle, pwm_params, servo_params, axis, leg)
    pca.servo[pwm_params.pins[axis, leg]].angle = joint_angle


def deactivate_servos(pca, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pca.channels[pwm_params.pins[axis_index, leg_index]].duty_cycle = 0

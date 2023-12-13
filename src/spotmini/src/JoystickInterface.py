import numpy as np
import time
from State import BehaviorState, State
from Command import Command
from Utilities import deadband, clipped_first_order_filter


class JoystickInterface:
    def __init__(
        self, config
    ):
        self.config = config
        self.previous_state = BehaviorState.REST


        # self.message_rate = 50

    def get_command(self, state, do_print=False):
        command = Command()
        
        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        trot = int(input("1 for trot, 0 for rest"))
        command.trot_event = (trot == 1)

        # Check if requesting a state transition to hopping, from trotting or resting
        command.hop_event = False           
        
        activate = int(input("1 for activate, 0 for disable"))
        command.activate_event = (activate == 1)

        ####### Handle continuous commands ########
        ly = int(input("Input y velocity [-1, 1]"))
        lx = int(input("Input x velocity [-1, 1]"))
        x_vel = ly * self.config.max_x_velocity
        y_vel = lx * -self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])

        rx = int(input("Input yaw rate [-1, 1]"))
        command.yaw_rate = rx * -self.config.max_yaw_rate

        # message_rate = int(input("Input message rate"))
        message_dt = 1.0 / 2.0

        pitch_angle = int(input("Input pitch angle [-1, 1]"))
        pitch = pitch_angle * self.config.max_pitch
        deadbanded_pitch = deadband(
            pitch, self.config.pitch_deadband
        )
        pitch_rate = clipped_first_order_filter(
            state.pitch,
            deadbanded_pitch,
            self.config.max_pitch_rate,
            self.config.pitch_time_constant,
        )
        command.pitch = state.pitch + message_dt * pitch_rate

        height_movement = int(input("Input height [-1, 1]"))
        command.height = state.height - message_dt * self.config.z_speed * height_movement
        
        roll_movement = - int(input("Input roll [-1, 1]"))
        command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement

        return command
#!/usr/bin/python

import odrive
from odrive.enums import *
import math
import time

class Motor:
    COUNTS_PER_REV = 4000 # Before gear box 
    GEAR_RATIO = 100

    Nm2A = 0.00000604

    def __init__(self, odrv, axis):
        self.odrv = odrv
        self.axis = self.odrv.axis0 if axis == 0 else self.odrv.axis1
        self.odrv.config.brake_resistance = 0
        self.axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.calibration_offset = 0 # Radians

    def set_angle(self, angle):
        self.axis.controller.pos_setpoint = self.rad2counts(angle + self.calibration_offset)
        #self.trajMoveRad(angle+self.calibration_offset)

    def set_angle_hard(self, angle):
        self.axis.controller.pos_setpoint = self.rad2counts(angle + self.calibration_offset)
    
    def trajMoveCnt(self, posDesired, velDesired, accDesired):
        #for ii in range(0,3):
        #self.axis = axes[ii]
        self.axis.trap_traj.config.vel_limit = 150000 #600000 max, 50000 is 1/8 rev per second
        self.axis.trap_traj.config.accel_limit = 100000 #50000 is 1/8 rev per second per second
        self.axis.trap_traj.config.decel_limit = 100000
        #print("Moved to position "+str(posDesired))#" with velocity "+str(velDesired)+" and accel "+str(accDesired))
        self.axis.controller.move_to_pos(posDesired) 

    def trajMoveRad(self, posDesired, velDesired = 2*math.pi/1.5, accDesired = 2*math.pi/1.0):
        self.trajMoveCnt(self.rad2counts(posDesired),  self.rad2counts(velDesired),  self.rad2counts(accDesired))
    
    def get_angle(self):
        return self.counts2rad(self.axis.encoder.pos_estimate) - self.calibration_offset
    
    def get_current(self):
        return self.axis.motor.current_control.Iq_measured
    
    def set_calibration(self, offset):
        self.calibration_offset = self.calibration_offset + offset

    def rad2counts(self, angle):
        return -angle*self.COUNTS_PER_REV*self.GEAR_RATIO/(2*math.pi)
    
    def counts2rad(self, counts):
        return -counts*2*math.pi/(self.COUNTS_PER_REV*self.GEAR_RATIO)

    def full_init(self):
        self.axis.motor.config.pre_calibrated = False
        self.axis.encoder.config.pre_calibrated = False

        #motor current limit
        self.axis.motor.config.current_lim = 5

        #pole pairs
        self.axis.motor.config.pole_pairs = 4

        self.axis.controller.config.vel_limit = 200000 #50000 counts/second is 1/8 revolution per second

        # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
        # Max speed is 1.35 Revolutions/second, or 539000counts/second
        self.axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        self.axis.encoder.config.cpr = self.COUNTS_PER_REV
        self.axis.encoder.config.bandwidth = 1000
        self.axis.encoder.config.use_index = True
        self.axis.encoder.config.zero_count_on_find_idx = True
        self.axis.encoder.config.idx_search_speed = 1
        self.axis.encoder.config.pre_calibrated = False

        #motor calibration current
        self.axis.motor.config.calibration_current = 5

        #axis state
        self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        print("Done doing setup.")
        time.sleep(20)
        print("Saving Configuration...")
        self.axis.motor.config.pre_calibrated = True
        self.axis.config.startup_encoder_index_search = True
        self.axis.config.startup_encoder_offset_calibration = True
        self.axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        #motor calibration current FOR INDEX SEARCH
        self.axis.motor.config.calibration_current = 5

        #Set closed loop gains
        kP_des = 16*self.Nm2A*100 # 
        kD_des = self.Nm2A*50  # 

        self.axis.controller.config.pos_gain = kP_des/kD_des #Convert to Cascaded Gain Structure
        #https://github.com/madcowswe/ODrive/blob/451e79519637fdcf33f220f7dae9a28b15e014ba/Firmware/MotorControl/controller.cpp#L151
        self.axis.controller.config.vel_gain = kD_des
        self.axis.controller.config.vel_integrator_gain = 0
        self.axis.controller.pos_setpoint = 0

        #axis state
        #odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis.config.startup_closed_loop_control = True
        # save configuration
        self.odrv.save_configuration()
        time.sleep(2)
        try:
            self.odrv.reboot()
        except:
            print('Rebooted 0')

        time.sleep(5)
        print("Done initializing! Reconnecting...")


        





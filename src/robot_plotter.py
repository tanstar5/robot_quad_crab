import time
import numpy as np
import pygame
from pygame.locals import *
from colorama import Fore,Back
from adafruit_servokit import ServoKit
from servo_motor import ServoMotor
#pv.set_plot_theme("dark")
from robot import Robot
from group_servos import GroupServos
from inverse_kinematics import InverseKinematics

class RobotClient():
    def __init__(self,robot:Robot):
        
        self.__window_name = "Robot State Window" 
        self.__robot_obj = robot

    def plot_leg(self,servo_grp_leg:GroupServos,
                 norm_pos_list:list[float],
                 arm_lengths:list[float]=[100,100,120],
                 leg_origins_polar:list[float]=[100,np.pi/2,np.pi/3],
                 leg_joint_gear_ratios=[-12/19,-12/19,-12/19],
                leg_joint_angle_offsets:list[float]=[10,45,0]
                ):

        servo_limits = [x.return_servo_limits() for x in self.__robot_obj.return_servo_list(grp=servo_grp_leg)]
        ik_obj = InverseKinematics(arm_lengths=arm_lengths,
                                   leg_origins_polar=leg_origins_polar,
                                   servo_limits=servo_limits,
                                   leg_joint_gear_ratios=leg_joint_gear_ratios,
                                   leg_joint_angle_offsets=leg_joint_angle_offsets)
        coors = np.array(ik_obj.calculate_leg_tip_coors(norm_pos_list=norm_pos_list))
        pygame.init()
        window = pygame.display.set_mode((600, 600))
        window.fill((255, 255, 255))
        pygame.draw.lines(surface=window,color=(255,0,0), points=coors[:,0:2],closed=False)
        pygame.display.update()
        


    

    
import time
import numpy as np
import socket
#import pygame
#from pygame.locals import *
from colorama import Fore,Back
from adafruit_servokit import ServoKit
from servo_motor import ServoMotor
#pv.set_plot_theme("dark")
#from robot import Robot
from group_servos import GroupServos
from inverse_kinematics import InverseKinematics
from robot_params import RobotParams

class RobotServer():
    def __init__(self,server_ip:str='192.168.0.114'):
        print(f'MESSAGE: Ronot server initiated')        
        self.__stateData = None
        #self.__robot_obj = robot
        self.__server_ip = server_ip
        self.__server_port = 2222
        self.__buffer_size = 1024
        self.__rpi_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.__rpi_socket.bind((server_ip,self.__server_port))
        self.__rpi_socket.setblocking(False)
        self.sync()

    def sync(self,servo_limits:list=None,servo_pos:np.ndarray=None):
        #servo_limits = self.__robot_obj.return_servo_limits()
        servo_limits = servo_limits
        #servo_pos = self.__robot_obj.return_norm_servo_positions()
        servo_pos = servo_pos
        self.__robo_params = RobotParams(servo_limits=servo_limits,servo_pos = servo_pos)

    def listen_from_client(self,client_address:str='192.168.0.160',servo_limits:list=None,servo_pos:np.ndarray=None):
        try:            
            message,address = self.__rpi_socket.recvfrom(self.__buffer_size)
        except BlockingIOError:
            #print(f'Message: No coms')
            pass
        else:
            if address[0] == client_address:
                self.send(client_address=address,servo_limits=servo_limits,servo_pos=servo_pos)
                print(f'Message: Packet sent to client {address}')
            else:
                print(f'WARNING: Unknown client trying to communicate {address[0]}')
        
        
    def send(self,client_address:tuple,servo_limits:list=None,servo_pos:np.ndarray=None):
        self.sync(servo_limits=servo_limits,servo_pos=servo_pos)
        packet = self.__robo_params.create_data_packet()
        self.__rpi_socket.sendto(packet,client_address)
        

class RobotClient():
    def __init__(self,server_address:str='192.168.0.114'):
        self.__server_address = server_address
        pass
    pass

if __name__ == '__main__':
    # servo_motor_00 = ServoMotor(ID=0,address=[ServoKit(channels=16,address=0x40),0],rest_position=95,limits=[40,150]) 
    # servo_motor_01 = ServoMotor(ID=1,address=[ServoKit(channels=16,address=0x40),1],rest_position=95,limits=[10,180])    
    # servo_motor_02 = ServoMotor(ID=2,address=[ServoKit(channels=16,address=0x40),2],rest_position=30,limits=[30,150])
    # servo_motor_10 = ServoMotor(ID=10,address=[ServoKit(channels=16,address=0x40),0+4],rest_position=75,limits=[20,130]) 
    # servo_motor_11 = ServoMotor(ID=11,address=[ServoKit(channels=16,address=0x40),1+4],rest_position=95,limits=[10,180])    
    # servo_motor_12 = ServoMotor(ID=12,address=[ServoKit(channels=16,address=0x40),2+4],rest_position=150,limits=[30,150])
    # servo_motor_20 = ServoMotor(ID=20,address=[ServoKit(channels=16,address=0x42),0],rest_position=75,limits=[20,130]) 
    # servo_motor_21 = ServoMotor(ID=21,address=[ServoKit(channels=16,address=0x42),1],rest_position=95,limits=[10,180])    
    # servo_motor_22 = ServoMotor(ID=22,address=[ServoKit(channels=16,address=0x42),2],rest_position=170,limits=[50,170])
    # servo_motor_30 = ServoMotor(ID=30,address=[ServoKit(channels=16,address=0x42),0+4],rest_position=115,limits=[60,170]) 
    # servo_motor_31 = ServoMotor(ID=31,address=[ServoKit(channels=16,address=0x42),1+4],rest_position=95,limits=[10,180])    
    # servo_motor_32 = ServoMotor(ID=32,address=[ServoKit(channels=16,address=0x42),2+4],rest_position=30,limits=[30,150])

    # servo_motor_list = np.array([ [servo_motor_00,servo_motor_01,servo_motor_02],
    #                              [servo_motor_10,servo_motor_11,servo_motor_12],
    #                               [servo_motor_20,servo_motor_21,servo_motor_22],
    #                                [servo_motor_30,servo_motor_31,servo_motor_32] ] )
    
    # rest_servo_pos = np.array([[0.5,0.5,0],
    #                            [.5,.5,1],
    #                             [.5,.5,1],
    #                              [.5,0.5,0]] )
    
    # robot_obj = Robot(servo_motor_list=servo_motor_list,rest_servo_pos=rest_servo_pos)
    # time.sleep(5)
    # robot_obj.arm()
    # robot_server = RobotServer(robot=robot_obj)
    # while(True):
    #     robot_server.listen_from_client(client_address='192.168.0.160')



    pass
        

    


    

    
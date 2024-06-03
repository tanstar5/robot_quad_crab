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
from robot_commands import RobotComms

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
    def __init__(self,client_address:str='192.168.0.160'):
        self.__client_address = client_address
        self.__client_port = 2221
        self.__buffer_size = 1024
        self.__client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.__client_socket.bind((self.__client_address,self.__client_port))
        self.__client_socket.setblocking(False)        
        self.__robot_params = RobotParams(servo_limits=None,servo_pos=None)
        
    def read_from_server(self,comm:RobotComms,server_address:tuple=('192.168.0.114',2222)):
        packet = comm.create_comm_packet(command=0)
        self.__client_socket.sendto(packet,server_address)
        time.sleep(.5)
        #packet_recv,address = self.__client_socket.recvfrom(self.__buffer_size)           
        #self.__robot_params.read_data_packet(packet=packet_recv)
        try:
           packet_recv,address = self.__client_socket.recvfrom(self.__buffer_size)           
           self.__robot_params.read_data_packet(packet=packet_recv)
           print(f'DEBUG: got  {self.__robot_params.__dict__} from server')
        except BlockingIOError:
           print(f'WARNING: server didnt respond {server_address} and robot_servo_pos is {self.__robot_params.servo_pos}')

if __name__ == '__main__':
    pass
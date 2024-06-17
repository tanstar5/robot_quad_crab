import numpy as np
import pickle as pickle
from copy import deepcopy

class RobotParams():
    def __init__(self,servo_limits:list[list]=[[40,150],[10,180],[30,150],
                                               [20,130],[10,180],[30,150],
                                               [20,130],[10,180],[50,170],
                                               [60,170],[10,180],[30,150]],
                                               servo_pos:list[list]=0.5*np.ones((4,3))):
        self.servo_limits = servo_limits
        self.servo_pos = servo_pos
        #self.__packet = None
        #self.command = None

    def update_servo_params(self,servo_pos:list[list]):
        self.servo_pos = servo_pos

    def create_data_packet(self):
        #print(f'DEBUG: dumping {self.__dict__} to self.__packet to send as packet')
        #param_in_dict = self.__dict__
        packet = pickle.dumps(self.__dict__,0)
        #self.__dict__ = {'servo_limits': [[4, 150], [10, 180], [30, 150]]}
        return packet
    
    def read_data_packet(self,packet):
        #self.__dict__ = pickle.loads(packet)
        comm_packet = deepcopy(pickle.loads(packet))
        command_code = comm_packet['command']
        #self.__dict__ = comm_packet['robot_params']
        #if command_code == 1:
        #    self.__dict__.update(comm_packet['_RobotComms__robot_params'])
        print(f'DEBUG: read {command_code},{comm_packet} to self.__packet to send as packet')
        return command_code ,deepcopy(comm_packet['robot_params'])
    
if __name__ == '__main__':
    robot_param_server = RobotParams(servo_limits=[[40,150],[10,180],[30,150]],servo_pos=[[0.5,0.5,0],
                               [.5,.5,1],
                                [.5,.5,1],
                                 [.5,0.5,0]])
    robot_param_client = RobotParams(servo_limits=None,servo_pos=None)    
    packet = robot_param_server.create_data_packet()
    print(robot_param_client.__dict__)
    robot_param_client.read_data_packet(packet=packet)
    print(robot_param_client.__dict__)


    
        
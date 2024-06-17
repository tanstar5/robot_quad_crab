import pickle as pickle
from robot_params import RobotParams

class RobotComms():
    def __init__(self,command:int=0):
        self.command = command
        self.__commands = {'read_robot_params':0,'write_robot_params':1}
        self.robot_params = None
    
    def create_comm_packet(self,command):
        self.command = command
        packet = pickle.dumps(self.__dict__,0)
        return packet
    
    def write_robot_params(self,robot_params:RobotParams):
        self.robot_params = robot_params

        
        
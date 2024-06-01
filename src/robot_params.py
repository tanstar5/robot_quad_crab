import pickle as pickle

class RobotParams():
    def __init__(self,servo_limits:list[list],servo_pos:list[list]):
        self.servo_limits = servo_limits
        self.servo_pos = servo_pos
        #self.__packet = None

    def update_servo_params(self,servo_pos:list[list]):
        self.servo_pos = servo_pos

    def create_data_packet(self):
        #print(f'DEBUG: dumping {self.__dict__} to self.__packet to send as packet')
        #param_in_dict = self.__dict__
        packet = pickle.dumps(self.__dict__,0)
        #self.__dict__ = {'servo_limits': [[4, 150], [10, 180], [30, 150]]}
        return packet
    
    def read_data_packet(self,packet):
        self.__dict__ = pickle.loads(packet)
        print(f'DEBUG: read {self.__dict__} to self.__packet to send as packet')
    
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


    
        
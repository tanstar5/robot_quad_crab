import pickle as pickle
class RobotComms():
    def __init__(self):
        self.command = None
    
    def create_comm_packet(self,command):
        self.command = command
        packet = pickle.dumps(self.__dict__,0)
        return packet
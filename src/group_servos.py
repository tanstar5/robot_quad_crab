import numpy as np
class GroupServos():
    def __init__(self,servo_index_mat:list[list[int]],relation:list,phase:list[float]):
        self.servo_index_mat = servo_index_mat
        self.__relation = relation
        self.__phase = phase
        self.__norm_pos = None
        #super().__init__(servo_motor_list=servo_motor_list,rest_servo_pos=rest_servo_pos)
        #self.__grp_servo_motors = [self.__servo_motor_list[servo_index[0]][servo_index[1]] for servo_index in self.__servo_index_mat]  
    
        
            
        
    def return_norm_pos(self,angular_pos:float,relation=None,phase=None):
        if relation!=None:
            self.__relation = relation
        if phase != None:
            self.__phase = phase    
        #grp_servo_motors = [self.__servo_motor_list[servo_index[0]][servo_index[1]] for servo_index in self.__servo_index_mat]
        norm_pos_list = [(np.sin(angular_pos*r+p)+1)/2 for r,p in zip(self.__relation,self.__phase)]
        #[servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in  zip(norm_pos_list,self.__grp_servo_motors)]
        return norm_pos_list
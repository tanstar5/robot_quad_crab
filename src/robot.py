import time
import numpy as np
from tqdm import tqdm
from colorama import Fore,Back
from adafruit_servokit import ServoKit
from servo_motor import ServoMotor
from group_servos import GroupServos
from robot_wifi import RobotServer


#from robot_leg import RobotLeg

class Robot(object):
    def __init__(self,servo_motor_list:np.ndarray[ServoMotor],rest_servo_pos:np.ndarray[float]):        
        self.__servo_motor_list = servo_motor_list
        self.__rest_servo_pos = rest_servo_pos
        self.__servo_pos = np.nan*np.ones(rest_servo_pos.shape)
        self.__server = RobotServer()

    def return_servo_list(self,grp:GroupServos):
        servo_motor_list = np.array([self.__servo_motor_list[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        return servo_motor_list
    
    def return_norm_servo_positions(self):
        return self.__servo_pos
    
    def return_servo_limits(self):
        return [x.return_servo_limits() for x in self.__servo_motor_list.flatten()]
    
    def extract_robot_params_from_server(self):
        return self.__server.get_robot_params_target()
    
    def listen_server(self):
        self.__server.listen_from_client(client_address='192.168.0.160',servo_limits=self.return_servo_limits(),servo_pos=self.__servo_pos)
        


    def arm(self):
        [servo_motor.arm() for servo_motor in self.__servo_motor_list.flatten()]
        [servo_motor.goto_norm_pos(norm_pos=rest_pos) for servo_motor,rest_pos in zip(self.__servo_motor_list.flatten(),self.__rest_servo_pos.flatten())]
        self.__servo_pos = self.__rest_servo_pos
        print(f'MESSAGE: Motor positions {self.__servo_pos}')

    def goto_smooth(self,position:np.ndarray[float],step_p_sec:float=10):
        step_p_ms = step_p_sec/1000
        current_pos = np.array(self.__servo_pos)
        displacement = np.array(position) - current_pos
        ds = np.sign(displacement)*step_p_ms
        max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten()))        
        while(max_diff>step_p_ms*1.5):
            current_pos = np.array(self.__servo_pos)
            displacement = np.round(np.array(position) - current_pos,4)
            ds = np.sign(displacement)*step_p_ms
            norm_pos_list = np.round(current_pos + ds,4)
            [servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in zip(norm_pos_list.flatten(),self.__servo_motor_list.flatten())]
            self.__servo_pos = norm_pos_list.tolist()
            max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten()))

    @staticmethod
    def update_mat_positions(mat:np.ndarray,updated_values:list[float|int],mat_index:list[list]):
        for val,index in zip(updated_values,mat_index):
            mat[index[0],index[1]] = val
        return mat

    def grp_goto_smooth(self,grp:GroupServos,angular_pos:float,step_p_sec:float=10,relation=None,phase=None):
        position = np.array(grp.return_norm_pos(angular_pos=angular_pos,relation=relation,phase=phase))
        servo_motor_list = np.array([self.__servo_motor_list[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        #print(f'DEBUG: self.__servo_pos = {self.__servo_pos}')
        current_pos = np.array([self.__servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        #print(f'DEBUG: current_posoutside = {current_pos}')
        step_p_ms = step_p_sec/1000
        #current_pos = np.array(self.__servo_pos)
        displacement = np.array(position) - current_pos
        ds = np.sign(displacement)*step_p_ms
        max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten())) 
        tot_displacement_mag = np.linalg.norm(displacement) 
        #pbar = tqdm(total=100,desc=Fore.GREEN+'Progress:')      
        while(max_diff>step_p_ms*1.5):
            #current_pos = np.array(self.__servo_pos)
            #print(f'DEBUG: current_pos = {current_pos}')
            displacement = np.round(np.array(position) - current_pos,4)
            ds = np.sign(displacement)*step_p_ms
            norm_pos_list = np.round(current_pos + ds,4)
            [servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in zip(norm_pos_list.flatten(),servo_motor_list.flatten())]
            #self.__servo_pos = norm_pos_list.tolist()
            self.__servo_pos = self.update_mat_positions(mat=self.__servo_pos,updated_values=norm_pos_list.tolist(),mat_index=grp.servo_index_mat)
            #print(f'DEBUG: position,current_pos,max_diff = {position,current_pos,max_diff}')
            current_pos = np.array([self.__servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
            #self.__servo_pos = self.update_mat_positions(mat=self.__servo_pos,updated_values=norm_pos_list.tolist(),mat_index=grp.servo_index_mat)
            max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(current_pos).flatten()))
            displacement_mag = np.linalg.norm(displacement)
            #print(f'DEBUG: displacement is {int((tot_displacement_mag-displacement_mag)/tot_displacement_mag*100)}')
            #pbar.update(int((tot_displacement_mag-displacement_mag)/tot_displacement_mag*100))
            self.__server.listen_from_client(client_address='192.168.0.160',servo_limits=self.return_servo_limits(),servo_pos=self.__servo_pos)
        #pbar.close()
        print(Fore.RESET+'MESSAGE: Reached to target')

    def grp_goto_smooth_from_client_target(self,robot_params_target ,grp:GroupServos,step_p_sec:float=10):
        
        position = np.array([robot_params_target.servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        #position = norm_pos_target
        servo_motor_list = np.array([self.__servo_motor_list[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        #print(f'DEBUG: self.__servo_pos = {self.__servo_pos}')
        current_pos = np.array([self.__servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
        #print(f'DEBUG: current_posoutside = {current_pos}')
        step_p_ms = step_p_sec/1000
        #current_pos = np.array(self.__servo_pos)
        displacement = np.array(position) - current_pos
        ds = np.sign(displacement)*step_p_ms
        max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten())) 
        tot_displacement_mag = np.linalg.norm(displacement) 
        #pbar = tqdm(total=100,desc=Fore.GREEN+'Progress:')      
        while(max_diff>step_p_ms*1.5):
            #current_pos = np.array(self.__servo_pos)
            #print(f'DEBUG: current_pos = {current_pos}')
            displacement = np.round(np.array(position) - current_pos,4)
            print(f'DEBUG: displacement is {displacement }')
            ds = np.sign(displacement)*step_p_ms
            norm_pos_list = np.round(current_pos + ds,4)
            [servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in zip(norm_pos_list.flatten(),servo_motor_list.flatten())]
            #self.__servo_pos = norm_pos_list.tolist()
            self.__servo_pos = self.update_mat_positions(mat=self.__servo_pos,updated_values=norm_pos_list.tolist(),mat_index=grp.servo_index_mat)
            #print(f'DEBUG: position,current_pos,max_diff = {position,current_pos,max_diff}')
            current_pos = np.array([self.__servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
            #self.__servo_pos = self.update_mat_positions(mat=self.__servo_pos,updated_values=norm_pos_list.tolist(),mat_index=grp.servo_index_mat)
            max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(current_pos).flatten()))
            displacement_mag = np.linalg.norm(displacement)
            #print(f'DEBUG: displacement is {int((tot_displacement_mag-displacement_mag)/tot_displacement_mag*100)}')
            #pbar.update(int((tot_displacement_mag-displacement_mag)/tot_displacement_mag*100))
            self.__server.listen_from_client(client_address='192.168.0.160',servo_limits=self.return_servo_limits(),servo_pos=self.__servo_pos)
            #robot_params_target = self.__server.get_robot_params_target()
            #print(f'DEBUG: robot_params_target is {robot_params_target.__dict__}')
            #position = np.array([robot_params_target.servo_pos[servo_index[0]][servo_index[1]] for servo_index in grp.servo_index_mat])
            time.sleep(.01)    
        #pbar.close()
        #print(Fore.RESET+'MESSAGE: Reached to target')

    def goto_smooth_server(self,position:np.ndarray[float],step_p_sec:float=10):
        #robot_params_target = robot_obj.extract_robot_params_from_server()
        step_p_ms = step_p_sec/1000
        current_pos = np.array(self.__servo_pos)
        position = np.reshape(position,newshape=current_pos.shape)
        displacement = np.array(position) - current_pos
        ds = np.sign(displacement)*step_p_ms
        max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten()))        
        while(max_diff>step_p_ms*1):
            current_pos = np.array(self.__servo_pos)
            displacement = np.round(np.array(position) - current_pos,4)
            ds = np.sign(displacement)*step_p_ms
            norm_pos_list = np.round(current_pos + ds,4)
            [servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in zip(norm_pos_list.flatten(),self.__servo_motor_list.flatten())]
            #self.__servo_pos = norm_pos_list.tolist()
            self.__servo_pos = norm_pos_list
            max_diff = max(abs(x-x0) for x,x0 in zip(position.flatten(),np.array(self.__servo_pos).flatten()))
            self.__server.listen_from_client(client_address='192.168.0.160',servo_limits=self.return_servo_limits(),servo_pos=self.__servo_pos)
        

    def stand(self):
        self.goto_smooth(position=np.array([[.5,0.8,0],
                               [.5,0.2,1],
                                [.5,0.2,1],
                                 [.5,.8,0]]),step_p_sec=50)
        time.sleep(1)
    def peek(self):
        self.goto_smooth(position=np.array([[.5,1,0],
                               [.5,0,1],
                                [.5,0,1],
                                 [.5,1,0]]),step_p_sec=50)
        time.sleep(1)
        
    
    def sit(self):
        self.goto_smooth(position=np.array([[.5,0.5,0],
                               [.5,.5,1],
                                [.5,.5,1],
                                 [.5,0.5,0]]),step_p_sec=50)
    
    def turn_left(self):
        self.goto_smooth(position=np.array([[.7,0.8,0],
                               [0.7,0.2,1],
                                [0.7,0.2,1],
                                 [0.7,.8,0]]),step_p_sec=50)
    def turn_right(self):
        self.goto_smooth(position=np.array([[.3,0.8,0],
                               [0.3,0.2,1],
                                [0.3,0.2,1],
                                 [0.3,.8,0]]),step_p_sec=50)
        
    def rotate_anticlockwise(self):
        self.goto_smooth(position=np.array([[.5,1,0],
                               [.5,.5,1],
                                [.5,0,1],
                                 [.5,.5,0]]),step_p_sec=50)
        time.sleep(1)
    def rotate_clockwise(self):
        self.goto_smooth(position=np.array([[.5,.5,0],
                               [.5,0,1],
                                [.5,0.5,1],
                                 [.5,1,0]]),step_p_sec=50)
        time.sleep(1)
        
    def disarm(self):
        [servo_motor.disarm() for servo_motor in self.__servo_motor_list.flatten()] 





if __name__ == '__main__':
    servo_motor_00 = ServoMotor(ID=0,address=[ServoKit(channels=16,address=0x40),0],rest_position=95,limits=[40,150]) 
    servo_motor_01 = ServoMotor(ID=1,address=[ServoKit(channels=16,address=0x40),1],rest_position=95,limits=[10,180])    
    servo_motor_02 = ServoMotor(ID=2,address=[ServoKit(channels=16,address=0x40),2],rest_position=30,limits=[30,150])

    servo_motor_10 = ServoMotor(ID=10,address=[ServoKit(channels=16,address=0x40),0+4],rest_position=75,limits=[20,130]) 
    servo_motor_11 = ServoMotor(ID=11,address=[ServoKit(channels=16,address=0x40),1+4],rest_position=95,limits=[10,180])    
    servo_motor_12 = ServoMotor(ID=12,address=[ServoKit(channels=16,address=0x40),2+4],rest_position=150,limits=[30,150])

    servo_motor_20 = ServoMotor(ID=20,address=[ServoKit(channels=16,address=0x42),0],rest_position=75,limits=[20,130]) 
    servo_motor_21 = ServoMotor(ID=21,address=[ServoKit(channels=16,address=0x42),1],rest_position=95,limits=[10,180])    
    servo_motor_22 = ServoMotor(ID=22,address=[ServoKit(channels=16,address=0x42),2],rest_position=170,limits=[50,170])

    servo_motor_30 = ServoMotor(ID=30,address=[ServoKit(channels=16,address=0x42),0+4],rest_position=115,limits=[60,170]) 
    servo_motor_31 = ServoMotor(ID=31,address=[ServoKit(channels=16,address=0x42),1+4],rest_position=95,limits=[10,180])    
    servo_motor_32 = ServoMotor(ID=32,address=[ServoKit(channels=16,address=0x42),2+4],rest_position=30,limits=[30,150])

    servo_motor_list = np.array([ [servo_motor_00,servo_motor_01,servo_motor_02],
                                 [servo_motor_10,servo_motor_11,servo_motor_12],
                                  [servo_motor_20,servo_motor_21,servo_motor_22],
                                   [servo_motor_30,servo_motor_31,servo_motor_32] ] )
    
    rest_servo_pos = np.array([[0.5,0.5,0],
                               [.5,.5,1],
                                [.5,.5,1],
                                 [.5,0.5,0]] )
    
    robot_obj = Robot(servo_motor_list=servo_motor_list,rest_servo_pos=rest_servo_pos)
    time.sleep(5)
    robot_obj.arm()

    step_p_sec=100   

    servo_grp_leg_all = \
              GroupServos(servo_index_mat=[[0,0],[0,1],[0,2],
                                           [1,0],[1,1],[1,2],
                                           [2,0],[2,1],[2,2],
                                           [3,0],[3,1],[3,2]],relation=[1,1,1,1,1,1,1,1,1,1,1,1],phase=0*np.array([1,1,1,1,1,1,1,1,1,1,1,1]))
          #robot_obj.grp_goto_smooth(grp=servo_grp_leg0_3_1 ,angular_pos=0,step_p_sec=step_p_sec,relation=np.array([1,-1,-1,1,-1,-1,-1,1,1]),phase=0*np.array([1,1,1,1,1,1,-1,-1,-1]))
    servo_index_mat=[[0,0],[0,1],[0,2],
                                           [1,0],[1,1],[1,2],
                                           [2,0],[2,1],[2,2],
                                           [3,0],[3,1],[3,2]]
    while(True):
        ####
        #robot_params_target = robot_obj.extract_robot_params_from_server()
        #robot_obj.listen_server()
        #robot_obj.grp_goto_smooth_from_client_target(robot_params_target=robot_params_target, grp=servo_grp_leg_all ,step_p_sec=step_p_sec)
        ###
        robot_params_target = robot_obj.extract_robot_params_from_server()
        robot_obj.listen_server()
        position = np.array([robot_params_target.servo_pos[servo_index[0]][servo_index[1]] for servo_index in servo_index_mat])
        print(f'MESSAGE: Target position is {position.tolist()}')
        robot_obj.goto_smooth_server(position=position,step_p_sec=step_p_sec)




    
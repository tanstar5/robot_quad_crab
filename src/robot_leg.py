import time
import numpy as np
from tqdm import tqdm
from adafruit_servokit import ServoKit
from servo_motor import ServoMotor

class RobotLeg():
    def __init__(self,leg_id:int,servo_motor_list:list[ServoMotor],leg_rest_servo_pos:list[float]):
        self.__ID = leg_id
        self.__servo_motor_list = servo_motor_list
        self.__rest_servo_pos = leg_rest_servo_pos
        self.__servo_pos = None

    
    def arm(self):
        [servo_motor.arm() for servo_motor in self.__servo_motor_list]
        [servo_motor.goto_norm_pos(norm_pos=rest_pos) for servo_motor,rest_pos in zip(self.__servo_motor_list,self.__rest_servo_pos)]
        self.__servo_pos = self.__rest_servo_pos
        print(f'MESSAGE: Leg {self.__ID} at Motor positions {self.__servo_pos}')

    def goto_norm_pos(self,norm_pos_list:list[float]):
        [servo_motor.goto_norm_pos(norm_pos) for norm_pos,servo_motor in zip(norm_pos_list,self.__servo_motor_list)]
        
    
    def goto_smooth(self,position:list[float],step_p_sec:float=10):
        step_p_ms = step_p_sec/1000/180
        current_pos = np.array(self.__servo_pos)
        displacement = np.array(position) - current_pos
        ds = np.sign(displacement)*step_p_ms
        max_diff = max(abs(x-x0) for x,x0 in zip(position,self.__servo_pos))
        print(f'DEBUG: ds is {ds} and max is {max_diff}')
        pbar = tqdm(total=100)
        max_diff_atStart = max_diff
        while(max_diff>step_p_ms):
            current_pos = np.array(self.__servo_pos)
            displacement = np.round(np.array(position) - current_pos,4)
            ds = np.sign(displacement)*step_p_ms
            norm_pos_list = np.round(current_pos + ds,4)
            [servo_motor.goto_norm_pos(norm_pos=norm_pos) for norm_pos,servo_motor in zip(norm_pos_list,self.__servo_motor_list)]
            self.__servo_pos = norm_pos_list.tolist()            
            max_diff = max(abs(x-x0) for x,x0 in zip(position,self.__servo_pos))
            pbar.update(100-int((max_diff_atStart-max_diff)/max_diff_atStart)*100)
        pbar.close()    
        


if  __name__ == '__main__':
    servo_motor_00 = ServoMotor(ID=0,address=[ServoKit(channels=16,address=0x40),0],rest_position=90,limits=[40,150]) 
    servo_motor_01 = ServoMotor(ID=1,address=[ServoKit(channels=16,address=0x40),1],rest_position=90,limits=[10,180])    
    servo_motor_02 = ServoMotor(ID=2,address=[ServoKit(channels=16,address=0x40),2],rest_position=90,limits=[30,150])

    servo_motor_10 = ServoMotor(ID=10,address=[ServoKit(channels=16,address=0x40),0+4],rest_position=55,limits=[20,130]) 
    servo_motor_11 = ServoMotor(ID=11,address=[ServoKit(channels=16,address=0x40),1+4],rest_position=90,limits=[10,180])    
    servo_motor_12 = ServoMotor(ID=12,address=[ServoKit(channels=16,address=0x40),2+4],rest_position=90,limits=[30,150])

    servo_motor_20 = ServoMotor(ID=20,address=[ServoKit(channels=16,address=0x42),0],rest_position=75,limits=[20,130]) 
    servo_motor_21 = ServoMotor(ID=21,address=[ServoKit(channels=16,address=0x42),1],rest_position=90,limits=[10,180])    
    servo_motor_22 = ServoMotor(ID=22,address=[ServoKit(channels=16,address=0x42),2],rest_position=90,limits=[50,170])

    servo_motor_30 = ServoMotor(ID=30,address=[ServoKit(channels=16,address=0x42),0+4],rest_position=90,limits=[60,170]) 
    servo_motor_31 = ServoMotor(ID=31,address=[ServoKit(channels=16,address=0x42),1+4],rest_position=90,limits=[10,180])    
    servo_motor_32 = ServoMotor(ID=32,address=[ServoKit(channels=16,address=0x42),2+4],rest_position=90,limits=[30,150])

    #leg_0 = RobotLeg(leg_id=0,servo_motor_list=[servo_motor_00,servo_motor_01,servo_motor_02],leg_rest_servo_pos=[90,10,30])
    #leg_1 = RobotLeg(leg_id=1,servo_motor_list=[servo_motor_10,servo_motor_11,servo_motor_12],leg_rest_servo_pos=[75,180,150])
    #leg_2 = RobotLeg(leg_id=2,servo_motor_list=[servo_motor_20,servo_motor_21,servo_motor_22],leg_rest_servo_pos=[75,180,170])
    #leg_3 = RobotLeg(leg_id=3,servo_motor_list=[servo_motor_30,servo_motor_31,servo_motor_32],leg_rest_servo_pos=[115,10,30])

    leg_0 = RobotLeg(leg_id=0,servo_motor_list=[servo_motor_00,servo_motor_01,servo_motor_02],leg_rest_servo_pos=[.5,0,0])
    leg_1 = RobotLeg(leg_id=1,servo_motor_list=[servo_motor_10,servo_motor_11,servo_motor_12],leg_rest_servo_pos=[.5,1,1])
    leg_2 = RobotLeg(leg_id=2,servo_motor_list=[servo_motor_20,servo_motor_21,servo_motor_22],leg_rest_servo_pos=[.5,1,1])
    leg_3 = RobotLeg(leg_id=3,servo_motor_list=[servo_motor_30,servo_motor_31,servo_motor_32],leg_rest_servo_pos=[.5,0,0])

    leg_0.arm()
    leg_1.arm()
    #leg_2.arm()
    #leg_3.arm()

    time.sleep(1)

    leg_0.goto_smooth(position=[0,0.5,0.5])
    leg_0.goto_smooth(position=[1,0,0])
    leg_0.goto_smooth(position=[.5,0,0])
    leg_0.goto_smooth(position=[.5,1,0])
    leg_0.goto_smooth(position=[.5,0,0])

    leg_1.goto_smooth(position=[1,0.5,0.5])
    leg_1.goto_smooth(position=[0,1,1])
    leg_1.goto_smooth(position=[.5,0,0])
    leg_1.goto_smooth(position=[.5,0,1])
    leg_1.goto_smooth(position=[.5,1,1])



    #leg_0.goto_norm_pos(norm_pos_list=[0.5,0.5,0])
    #leg_1.goto_norm_pos(norm_pos_list=[0.5,0.5,1])
    #leg_2.goto_norm_pos(norm_pos_list=[0.5,0.5,1])
    #leg_3.goto_norm_pos(norm_pos_list=[0.5,0.5,0])

    #leg_0.goto_norm_pos(norm_pos_list=[0.5,1,0])
    #leg_1.goto_norm_pos(norm_pos_list=[0.5,0,1])
    #leg_2.goto_norm_pos(norm_pos_list=[0.5,0,1])
    #leg_3.goto_norm_pos(norm_pos_list=[0.5,1,0])

    #leg_0.goto_norm_pos(norm_pos_list=[0.5,0,0])
    #leg_1.goto_norm_pos(norm_pos_list=[0.5,1,1])
    #leg_2.goto_norm_pos(norm_pos_list=[0.5,1,1])
    #leg_3.goto_norm_pos(norm_pos_list=[0.5,0,0])
    

    



        
    
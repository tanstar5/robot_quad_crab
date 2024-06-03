import time
from colorama import Fore,Back
import numpy as np
from adafruit_servokit import ServoKit

class ServoMotor():
    def __init__(self,ID:int,address:list[ServoKit,int],rest_position:float,limits:list[float,float]):
        self.__ID = ID
        self.__control_pin = address[1]        
        if rest_position>=limits[0] and rest_position<=limits[1]:
            self.__rest_position = rest_position
        else:
            print(f'MESSAGE: init failed as rest_position {rest_position} is not in range {limits}')
            raise ValueError
        self.__limits = limits
        self.__kit = address[0]
        self.__position = None

    def return_servo_limits(self):
        return self.__limits

    def arm(self):        
        self.__kit.servo[self.__control_pin].angle = self.__rest_position
        time.sleep(.1)
        print(f'MESSAGE: Servo {self.__ID} is armed and resting at {self.__rest_position}')
        self.__position = self.__rest_position
    
    def disarm(self):
        self.__kit.servo[self.__control_pin].angle = None
        time.sleep(1)
        print(f'MESSAGE: Servo {self.__ID} is disarmed and was resting at {self.__position}')
        self.__rest_position = self.__position
        self.__position = None
    
    def goto(self,position):
        if self.__position:
            #step_p_ms = step_p_sec/1000
            #displacement = position-self.__position
            #ds  = np.sign(displacement)*step_p_ms
            pos_to_go = position 
            if pos_to_go>=self.__limits[0] and pos_to_go<=self.__limits[1]:
                self.__kit.servo[self.__control_pin].angle = pos_to_go
                self.__position = pos_to_go
                time.sleep(.01)
            else:
                print(Back.YELLOW + 'WARNING:' + Back.RESET + f'Position {pos_to_go} out of motor limits and therefore stopped at {self.__position}')
                                
            
        else:
            print(f'MESSAGE: Motor is disarmed! Arm it before moving')
    
    def goto_norm_pos(self,norm_pos:float=0.5):
        min_pos = self.__limits[0]
        max_pos = self.__limits[1]
        norm_pos_correct = lambda norm_pos: 0 if norm_pos<0 else norm_pos if norm_pos<1 else 1 
        

        if norm_pos<=1 and norm_pos>=0:
            pos_to_go = min_pos + (max_pos-min_pos)*norm_pos
            self.goto(position=pos_to_go)
        else:
            print(f'WARNING: norm_pos {norm_pos} not in range [0,1]')
            norm_pos = norm_pos_correct(norm_pos=norm_pos)
            print(f'WARNING: norm_pos corrected to {norm_pos} not in range [0,1]')
            pos_to_go = min_pos + (max_pos-min_pos)*norm_pos
            self.goto(position=pos_to_go)
            #raise ValueError
        pass

    def goto_smooth(self,position,step_p_sec:float=800):        
        if self.__position:
            step_p_ms = step_p_sec/1000
            displacement = position-self.__position
            ds  = np.sign(displacement)*step_p_ms
            while(abs(self.__position-position)>=1):
                pos_to_go = self.__position + ds
                if pos_to_go>=self.__limits[0] and pos_to_go<=self.__limits[1]:
                    self.__kit.servo[self.__control_pin].angle = pos_to_go
                    self.__position = pos_to_go
                    time.sleep(.001)
                else:
                    print(Back.YELLOW + 'WARNING:' + Back.RESET + f'Position {pos_to_go} out of motor limits and therefore stopped at {self.__position}')
                    break
            
        else:
            print(f'MESSAGE: Motor is disarmed! Arm it before moving')
        

        
    

if __name__ == '__main__':
    servo_motor_01 = ServoMotor(ID=1,address=[ServoKit(channels=16,address=0x40),1],rest_position=90,limits=[10,180])    
    servo_motor_02 = ServoMotor(ID=2,address=[ServoKit(channels=16,address=0x40),2],rest_position=90,limits=[30,150])
    servo_motor_01.arm();servo_motor_02.arm()
    servo_motor_02.goto_smooth(position=30,step_p_sec=500);servo_motor_01.goto_smooth(position=180,step_p_sec=500)
    #servo_motor_01.goto(position=9);servo_motor_02.goto(position=120)
    servo_motor_01.goto_smooth(position=90,step_p_sec=100);servo_motor_02.goto_smooth(position=90,step_p_sec=100)
    servo_motor_01.disarm();servo_motor_02.disarm()

                             
    pass
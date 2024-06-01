import time
import numpy as np
from tqdm import tqdm
from adafruit_servokit import ServoKit
from colorama import Fore, Back, Style

MOTORS:list = [0,1,2]
MIDVALUE:float = 90

class servo_leg(object):
    def __init__(self) -> None:
        self.__kit = [ServoKit(channels=16,address=0x40),
                      ServoKit(channels=16,address=0x42)]
        
    
    def power_off_legs(self):
        for leg in [0,1,2]:
            self.power_off_leg(leg_num=leg)
            
    def goto(self,leg_num:int,motor:int,position:float):
        if leg_num<=1:
            self.__kit[0].servo[motor+4*leg_num].angle = position
        else:
            self.__kit[1].servo[motor+4*(leg_num-2)].angle = position

    def set_leg_mid_config(self,leg_num:int,mid_pos:float=90):
        [self.goto(leg_num=leg_num,motor=x,position=mid_pos) for x in MOTORS]
        
    def set_legs_mid_config(self,leg_nums:list,mid_pos:float=90):
        [self.set_leg_mid_config(leg_num=x,mid_pos=mid_pos) for x in leg_nums]


    def rotate_leg_motor(self,leg_num:int, motor:int, shift_by:float):
        self.goto(leg_num=leg_num,motor=motor,position=MIDVALUE+shift_by)
        pass   

    def power_off_leg(self,leg_num:int):
        for motor in [0,1,2]:
            self.__kit.servo[motor+4*leg_num].angle = None
        

    def sweep_leg_motor(self,leg_num:int,motor_num:int,step:int=5):
        for angle in np.arange(0,180,step):
            self.__kit.servo[motor_num+4*leg_num].angle = angle
            time.sleep(0.01)
            
        for angle in np.arange(180,0,-step):
            self.__kit.servo[motor_num+4*leg_num].angle = angle
            time.sleep(0.05)
        self.power_off_legs()

    def sweep_legs(self,leg_nums:list[int],step:int=1,limit:list[float]=[0,180]):
        for motor_pin in [0,1,2]:
            if min(limit) != limit[0]:
                step = -abs(step)
            for angle in tqdm(np.arange(limit[0],limit[1],step),
                              desc=Fore.GREEN+Back.BLACK+f"Moving motor {motor_pin}"+Fore.RESET+Back.RESET):
                for leg in leg_nums:
                    if leg%2==0:
                        self.__kit.servo[motor_pin+4*leg].angle = angle
                        time.sleep(0.001)
                    else:
                        self.__kit.servo[motor_pin+4*leg].angle = 180-angle
                        time.sleep(0.001)
                time.sleep(0.01)
            self.power_off_legs()
            time.sleep(0.1)
        
        



if __name__ == '__main__':
    servo_leg_obj = servo_leg()
    time.sleep(5)
    #servo_leg_obj.sweep_leg_motor(leg_num=1,motor_num=1)
    #servo_leg_obj.power_off_legs()
    #servo_leg_obj.goto(leg_num=0,motor=1,position=150)
    for i in range(4):
        servo_leg_obj.rotate_leg_motor(leg_num=0,motor=1,shift_by=60)
        servo_leg_obj.rotate_leg_motor(leg_num=0,motor=2,shift_by=-60)
        servo_leg_obj.rotate_leg_motor(leg_num=1,motor=1,shift_by=-60)
        servo_leg_obj.rotate_leg_motor(leg_num=1,motor=2,shift_by=60)

        servo_leg_obj.rotate_leg_motor(leg_num=2,motor=1,shift_by=-60)
        servo_leg_obj.rotate_leg_motor(leg_num=2,motor=2,shift_by=60)
        servo_leg_obj.rotate_leg_motor(leg_num=3,motor=1,shift_by=60)
        servo_leg_obj.rotate_leg_motor(leg_num=3,motor=2,shift_by=-60)

        time.sleep(2)
        servo_leg_obj.set_legs_mid_config(leg_nums=[0,1,2,3])
        time.sleep(2)
    #servo_leg_obj.power_off_legs()
    #servo_leg_obj.sweep_legs(leg_nums=[0])
    
    
    
    
    
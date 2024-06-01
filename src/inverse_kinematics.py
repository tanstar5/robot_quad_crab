import numpy as np

class InverseKinematics():
    def __init__(self,arm_lengths:list[float]=[100,100,120],
                 leg_origins_polar:list[float]=[100,np.pi/2,np.pi/3], servo_limits:list[list]=[[40,150],[10,180],[30,150]],
                                        leg_joint_gear_ratios:list[float]=[-12/19,-12/19,-12/19],
                                        leg_joint_angle_offsets:list[float]=[10,45,0],                                                                        
                                                                            ):
        self.__arm_lengths = arm_lengths
        self.__leg_origins_polar = leg_origins_polar
        self.__servo_limits = servo_limits
        self.__leg_joint_gear_ratios = leg_joint_gear_ratios
        self.__leg_joint_angle_offsets = leg_joint_angle_offsets
        

    def calculate_leg_tip_coors(self,norm_pos_list:list[float]):
        #num_legs = len(self.__leg_origins_polar)
        leg_origin_polar = self.__leg_origins_polar
        leg_origin_cartesian = self.polar2cart(r=leg_origin_polar[0],theta=leg_origin_polar[1],phi=leg_origin_polar[2])
        leg_joint_gear_ratios = self.__leg_joint_gear_ratios
        servo_limits = self.__servo_limits
        leg_joint_angle_offsets = self.__leg_joint_angle_offsets
        leg_joint_coors = self.leg_tip_coor_calculation_method(leg_origin_cartesian=leg_origin_cartesian,
                                             norm_pos_list=norm_pos_list,
                                             servo_limits=servo_limits,
                                             leg_joint_angle_offsets=leg_joint_angle_offsets,
                                             leg_joint_gear_ratios=leg_joint_gear_ratios)
        return leg_joint_coors
    
    
    def leg_tip_coor_calculation_method(self,
                                        leg_origin_cartesian:list[float],                                        
                                        norm_pos_list:list[float],
                                        #servo_relations:list[float],
                                        #servo_phase:list[float],
                                        servo_limits:list[list],
                                        leg_joint_gear_ratios:list[float]=[-12/19,-12/19,-12/19],
                                        leg_joint_angle_offsets:list[float]=[10,45,0],
                                        ):
        leg_arm_lengths = self.__arm_lengths
        servo_0__norm_angle = norm_pos_list[0]
        arm_angle_drive_0 = self.get_leg_angle_from_servo_norm_pos(norm_pos=servo_0__norm_angle,servo_limits=servo_limits[0],gear_ratio=leg_joint_gear_ratios[0])
        
        arm_tip_1_x = leg_origin_cartesian[0] + leg_arm_lengths[0]*np.cos(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_1_y = leg_origin_cartesian[1] + leg_arm_lengths[0]*np.sin(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_1_z = leg_origin_cartesian[2]

        servo_1__norm_angle = norm_pos_list[1]
        arm_angle_drive_1 = self.get_leg_angle_from_servo_norm_pos(norm_pos=servo_1__norm_angle,servo_limits=servo_limits[1],gear_ratio=leg_joint_gear_ratios[1])
        arm_2_base = leg_arm_lengths[1]*np.cos(leg_joint_angle_offsets[1]+arm_angle_drive_1)
        arm_2_perpendicular = leg_arm_lengths[1]*np.sin(leg_joint_angle_offsets[1]+arm_angle_drive_1)
        
        arm_tip_2_x = arm_tip_1_x + arm_2_base*np.cos(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_2_y = arm_tip_1_y + arm_2_base*np.sin(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_2_z = arm_tip_1_z + arm_2_perpendicular

        servo_2__norm_angle = norm_pos_list[2]
        arm_angle_drive_2 = self.get_leg_angle_from_servo_norm_pos(norm_pos=servo_2__norm_angle,servo_limits=servo_limits[2],gear_ratio=leg_joint_gear_ratios[2])
        arm_3_base = leg_arm_lengths[2]*np.cos(leg_joint_angle_offsets[2]+arm_angle_drive_2)
        arm_3_perpendicular = leg_arm_lengths[2]*np.sin(leg_joint_angle_offsets[2]+arm_angle_drive_2)

        arm_tip_3_x = arm_tip_2_x + arm_3_base*np.cos(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_3_y = arm_tip_2_y + arm_3_base*np.sin(leg_joint_angle_offsets[0]+arm_angle_drive_0)
        arm_tip_3_z = arm_tip_2_z + arm_3_perpendicular

        return [leg_origin_cartesian,[arm_tip_1_x,arm_tip_1_y,arm_tip_1_z],[arm_tip_2_x,arm_tip_2_y,arm_tip_2_z],[arm_tip_3_x,arm_tip_3_y,arm_tip_3_z]]

        

        

    
    @staticmethod
    def get_norm_servo_pos_from_angular_pos(angular_pos:float,relation:float,phase:float):
        norm_servo_pos = (np.sin(angular_pos*relation+phase)+1)/2
        return norm_servo_pos
    
    @staticmethod
    def get_leg_angle_from_servo_norm_pos(norm_pos:float,servo_limits:list[float],gear_ratio:float=-12/19):
        angle_per_norm_value = servo_limits[1]-servo_limits[0]
        angle_rotated_in_servo = angle_per_norm_value*norm_pos
        angle_rotated_by_arm = angle_rotated_in_servo*gear_ratio
        return angle_rotated_by_arm
    
    @staticmethod
    def polar2cart(r:float, theta:float, phi:float):
        return [
            r * np.sin(theta) * np.cos(phi),
            r * np.sin(theta) * np.sin(phi),
            r * np.cos(theta)
        ]
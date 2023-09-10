import pybullet as p
import pybullet_data
import time
import os 
import sys

# Get the absolute path of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the sources directory to the system path
sources_dir = os.path.join(current_dir, '.')
sys.path.append(sources_dir)


from simulator.pybullet.simulation_pybullet import SimulationPybullet

from utils.biped_robot import BipedRobot

from kinematics.kdl_kinematics import KDLKinematics


from utils.biped_model import BlackBirdModel

import numpy as np
    
if __name__ == '__main__':
    print(sys.path)
    print("********************************")
    robot_urdf_path = "../config/models/blackbird_urdf/urdf/blackbird.urdf"
    sim = SimulationPybullet(robot_urdf_path)
    
    robot = BipedRobot("robot", None)

    kdl_model = BlackBirdModel("blackbm")

    '''
    left leg
    '''
    left_leg_chain = kdl_model.left_leg_chain
    print(left_leg_chain)
    nrof_left_leg_joints = kdl_model.nrof_left_leg_joints
    qmin_left_leg_joints = [-np.pi]*nrof_left_leg_joints
    qmax_left_leg_joints = [np.pi]*nrof_left_leg_joints
    init_angles = [0]*nrof_left_leg_joints 
    left_leg_kinematics = KDLKinematics(left_leg_chain)
    init_angles = [0]*left_leg_kinematics.num_jnts
    init_left_leg_position = left_leg_kinematics.forward_kinematics(init_angles)
    print(init_left_leg_position)

    target_left_leg_position = init_left_leg_position.copy()
    angles = left_leg_kinematics.inverse_kinematics(target_left_leg_position)
    print(angles)
    print(left_leg_kinematics.forward_kinematics(angles))


    print(left_leg_chain.getNrOfJoints())

    while(1):
        
        keys = p.getKeyboardEvents()    
        for k,v in keys.items():
            
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_IS_DOWN)):
                target_left_leg_position[1]  += 0.001
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_IS_DOWN)):
                target_left_leg_position[1] -= 0.001
            if (k == p.B3G_UP_ARROW and (v&p.KEY_IS_DOWN)):
                target_left_leg_position[0] += 0.001
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_IS_DOWN)):
                target_left_leg_position[0] -= 0.001
            if (k == 65309 and (v&p.KEY_WAS_TRIGGERED)): # enter key
                print("pushed enter")
            if (k == p.B3G_CONTROL and (v&p.KEY_IS_DOWN)):
                print("ctl")
                target_left_leg_position[2] += 0.001
            if (k == p.B3G_ALT and (v&p.KEY_IS_DOWN)):
                target_left_leg_position = init_left_leg_position.copy()
                print("alt")
            print(k)


        new_angles = left_leg_kinematics.inverse_kinematics(target_left_leg_position)
        if new_angles.all() != None:
            angles = new_angles
        else:
            print("An error returns not to the new state.")

   
        for i, angle in enumerate(angles):
            sim.setDesiredMotorAngleById(i, angle)


        
        sim.step()


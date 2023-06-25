import pybullet as p
import pybullet_data
import time
import os 
import sys

# Get the absolute path of the current script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Add the sources directory to the system path
sources_dir = os.path.join(current_dir, 'src')
sys.path.append(sources_dir)



from simulator.pybullet.simulation_pybullet import SimulationPybullet

from utils.biped_robot import BipedRobot

from kinematics.kdl_kinematics import KDLKinematics


from utils.biped_model import BlackBirdModel

import numpy as np
    
if __name__ == '__main__':
    robot_urdf_path = "./config/models/blackbird_urdf/urdf/blackbird.urdf"
    sim = SimulationPybullet(robot_urdf_path)
    
    robot = BipedRobot("robot", None)

    kdl_model = BlackBirdModel("blackbm")

    left_leg_chain = kdl_model.left_leg_chain
    print(left_leg_chain)
    nrof_left_leg_joints = kdl_model.nrof_left_leg_joints
    qmin_left_leg_joints = [-np.pi]*nrof_left_leg_joints
    qmax_left_leg_joints = [np.pi]*nrof_left_leg_joints
    init_angles = [0]*nrof_left_leg_joints 
    left_leg_kinematics = KDLKinematics(left_leg_chain,qmin_left_leg_joints,qmax_left_leg_joints)

    init_left_leg_position = left_leg_kinematics.forward_kinematics(init_angles)[:3,3]
    print(init_left_leg_position)

    target_left_leg_position = init_left_leg_position
    angles = left_leg_kinematics.inverse_kinematics(target_left_leg_position)
    print(angles)
    print(left_leg_kinematics.forward_kinematics(angles))


    while(1):
        
        
        keys = p.getKeyboardEvents()    
        arrow = [0.,0.,0.]
        for k,v in keys.items():
            
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_IS_DOWN)):
                arrow[1] = 0.001
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_IS_DOWN)):
                arrow[1] = -0.001
            if (k == p.B3G_UP_ARROW and (v&p.KEY_IS_DOWN)):
                arrow[0] = 0.001
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_IS_DOWN)):
                arrow[0] = -0.001
            if (k == 65309 and (v&p.KEY_WAS_TRIGGERED)): # enter key
                print("pushed enter")
            if (k == p.B3G_CONTROL and (v&p.KEY_IS_DOWN)):
                print("ctl")
                arrow[2] += 0.001
            if (k == p.B3G_ALT and (v&p.KEY_IS_DOWN)):
                target_left_leg_position = init_left_leg_position
                print("alt")
            print(k)
        for i in range(len(arrow)):
            target_left_leg_position[i] += arrow[i]

        new_angles = left_leg_kinematics.inverse_kinematics(target_left_leg_position)
        if new_angles.all() == None:
            #retun to previous state
            print("An error returns to the previous state.")
            for i in range(len(arrow)):
                target_left_leg_position[i] -= arrow[i]
        else:
            angles = new_angles


            
        for i, angle in enumerate(angles):
            sim.setDesiredMotorAngleById(i, angle)


        
        sim.step()


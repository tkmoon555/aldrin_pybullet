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
    nrof_left_leg_joints = kdl_model.nrof_left_leg_joints
    qmin_left_leg_joints = [-np.pi/2]*nrof_left_leg_joints
    qmax_left_leg_joints = [np.pi/2]*nrof_left_leg_joints
    init_angles = [0]*nrof_left_leg_joints 
    left_leg_kinematics = KDLKinematics(left_leg_chain,qmin_left_leg_joints,qmax_left_leg_joints)

    init_left_leg_position = left_leg_kinematics.forward_kinematics(init_angles)[:3,3]
    print(init_left_leg_position)

    while(1):
        sim.step()


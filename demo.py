import pybullet as p
import pybullet_data
import time
import os 
import sys

sys.path.append(os.path.abspath("./scripts/"))
from simulator.pybullet.simulation_pybullet import SimulationPybullet

sys.path.append(os.path.abspath("./scripts/utils"))
from utils.biped_robot import BipedRobot

sys.path.append(os.path.abspath("./scripts/kinematics"))
from kinematics.kdl_kinematics

if __name__ == '__main__':
    pkg_path = os.path.abspath('.')
    sim = SimulationPybullet(pkg_path)
    
    robot = BipedRobot()
    while(1):
        sim.step()

import pybullet as p
import pybullet_data
import time
import os 
import sys


from biped_model import BipedModel

from simulation_pybullet import SimulationPybullet

if __name__ == '__main__':
    sim = SimulationPybullet()

    sim.run()

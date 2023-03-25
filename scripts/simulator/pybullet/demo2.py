import pybullet as p
import pybullet_data
import time
import os 
import sys

sys.path.append(os.path.abspath("../../utils"))
from biped_model import BipedModel
try:
    p.connect(p.GUI)  # Connect to the PyBullet physics server
except p.error as e:
    # if handle the error, connect to xserver
    p.connect(p.DIRECT)  # Connect to the PyBullet physics server
finally:
    # Code to execute regardless of whether an error occurred
    print("This will always execute")
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

useFixedBase = True
flags = p.URDF_INITIALIZE_SAT_FEATURES
print(os.path.dirname(__file__))
ground_pos = [0,0,-0.625]
ground = p.loadURDF("../../../config/world/ground.urdf", ground_pos, flags = flags, useFixedBase=useFixedBase)
robot_pos = [0,0,0.5]
robot_urdf_path = '../../../config/models/simplebot_v10/model.urdf'
robot_id = p.loadURDF(robot_urdf_path, robot_pos)


p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(robot_id, -1, linearDamping=0, angularDamping=0)


p.setGravity(0, 0,-0.1)  # Set the gravity to -9.81 m/s^2 in the z-direction
p.setTimeStep(1.0/240.0)   # Set the time step to 1/240 seconds
p.setRealTimeSimulation(1)

model = BipedModel(robot_urdf_path)
print(model.COG())


#for i in range(10000):  # Run the simulation for 1000 steps
while(1):
    #print("*************************")
    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(robot_id)  # Get the position and orientation of the robot's base link
    #print("Robot position:", pos)
    #print("Robot orientation:", orn)

#p.disconnect()

import pybullet as p
import pybullet_data
import time
import os 

def control_joint_pos(body, joint, pos, max_vel=None, max_force=None):
        if max_vel is None:
            p.setJointMotorControl2(
                    bodyIndex=body,
                    jointIndex=joint,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=pos,
                    force=max_force)
        else:
            p.setJointMotorControl2(
                    bodyIndex=body,
                    jointIndex=joint,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=pos,
                    targetVelocity=max_vel,
                    force=max_force)

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
#print("***************************")#ok
ground_pos = [0,0,-0.0]#ground_pos = [0,0,-0.625]
ground = p.loadURDF("../../../config/world/ground.urdf", ground_pos, flags = flags, useFixedBase=useFixedBase)
robot_pos = [0,0,1]
#print("***************************")#
robot_id = p.loadURDF("../../../config/models/robot_assy_v1/model.urdf", robot_pos)#, flags = p.URDF_USE_INERTIA_FROM_FILE)
#print("***************************")#x
p.setPhysicsEngineParameter(numSolverIterations=10)
#p.changeDynamics(robot_id, -1, linearDamping=0, angularDamping=0)

p.setGravity(0, 0,-0.5)  # Set the gravity to -9.81 m/s^2 in the z-direction
#p.setTimeStep(1.0/240.0)   # Set the time step to 1/240 seconds
p.setRealTimeSimulation(1)


joint_ids = []
param_ids = []
for j in range(p.getNumJoints(robot_id)):
    p.changeDynamics(robot_id, j, angularDamping=0)
    p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)

    info = p.getJointInfo(robot_id, j)
    jointName = info[1]
    jointType = info[2]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        joint_ids.append(j)
        param_ids.append(p.addUserDebugParameter(jointName.decode("utf-8"), jointLowerLimit, jointUpperLimit, 0))
        print("joint ID : {}, Name : {}".format(j, jointName.decode("utf-8")))

import numpy as np
kosi = 20*np.pi/180
hiza = 85*np.pi/180
ankle = 90*np.pi/180
max_vel = None
force = 5.*1 # Nm



count = 0
start = 1
flag = 0
#for i in range(10000):  # Run the simulation for 1000 steps
while(1):
    
    #print("*************************")
    pos, orn = p.getBasePositionAndOrientation(robot_id)  # Get the position and orientation of the robot's base link
    #print("Robot position:", pos)
    #print("Robot orientation:", orn)
    if count < start:
        pass
    else:
        if flag == 0:
            #jump
            control_joint_pos(robot_id, joint_ids[0],-kosi,max_vel,force)
            control_joint_pos(robot_id, joint_ids[1],-hiza,max_vel,force)
            control_joint_pos(robot_id, joint_ids[2],ankle,max_vel,force)
            control_joint_pos(robot_id, joint_ids[3],kosi,max_vel,force)
            control_joint_pos(robot_id, joint_ids[4],-hiza,max_vel,force)
            control_joint_pos(robot_id, joint_ids[5],-ankle,max_vel,force)

        elif flag == 1:
            for i in range(len(param_ids)):
                c = param_ids[i]
                targetPos = p.readUserDebugParameter(c) *1
                p.setJointMotorControl2(robot_id, joint_ids[i], p.POSITION_CONTROL, targetPos, force=force)
                #p.setJointMotorControl2(robot_id, joint_ids[i], p.TORQUE_CONTROL, force = targetPos)
            
        else: 
            print("no flag")
        
        pressed_keys = []
        events = p.getKeyboardEvents()
        key_codes = events.keys()
        for key in key_codes:
            pressed_keys.append(key)  
            #print(targetPos)
            if chr(key) =='0':
                flag = 0
            elif chr(key) == '1':
                flag = 1

    count += 1
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()


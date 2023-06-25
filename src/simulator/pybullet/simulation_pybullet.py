import time
import numpy as np
import pybullet as p
import pybullet_data

class SimulationPybullet:
    def __init__(self,robot_urdf_path):

        self._robot_urdf_path = robot_urdf_path
        self.reset()
    def __del__(self):
        p.disconnect()
    def _set_server(self):
        try:
            p.connect(p.GUI)  # Connect to the PyBullet physics server
        except p.error as e:
            # if handle the error, connect to xserver
            p.connect(p.DIRECT)  # Connect to the PyBullet physics server
        finally:
            # Code to execute regardless of whether an error occurred
            print("This will always execute")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    def _set_model(self):
        useFixedBase = True
        flags = p.URDF_INITIALIZE_SAT_FEATURES
        ground_pos = [0,0,-0.625]
        ground = p.loadURDF("./config/world/ground.urdf", ground_pos, flags = flags, useFixedBase=useFixedBase)
        robot_pos = [0,0,0.5]
        robot_id = p.loadURDF(self._robot_urdf_path, robot_pos,useFixedBase=1)
        joint_ids = []
         
        param_ids = []
        for j in range(p.getNumJoints(robot_id)):
            p.changeDynamics(robot_id, j, angularDamping=0.5)
            p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=100)

            info = p.getJointInfo(robot_id, j)
            jointName = info[1]
            jointType = info[2]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                joint_ids.append(j)
                param_ids.append(p.addUserDebugParameter(jointName.decode("utf-8"), jointLowerLimit, jointUpperLimit, 0))
                print("joint ID : {}, Name : {}".format(j, jointName.decode("utf-8")))
    
        self._robot_id = robot_id
        self._motor_id_list = joint_ids
        self._param_id_list = param_ids
    
    def _set_environment_param(self):
        p.setPhysicsEngineParameter(numSolverIterations=10)
        #p.changeDynamics(robot_id, -1, linearDamping=0, angularDamping=0)
        p.setGravity(0, 0,-0)  # Set the gravity to -9.81 m/s^2 in the z-direction
        p.setTimeStep(1.0/240.0)   # Set the time step to 1/240 seconds
        p.setRealTimeSimulation(0)
    
    def reset(self):
        if p.isConnected(): p.resetSimulation()

        self._set_server()
        self._set_model()
        self._set_environment_param()



    def step(self):
        #for i in range(10000):  # Run the simulation for 1000 steps
        #while(p.isConnected()):
        if not (p.isConnected()): exit()

        p.stepSimulation()
        """
        pos, orn = p.getBasePositionAndOrientation(self._robot_id)  # Get the position and orientation of the robot's base link
        for i in range(len(self._param_id_list)):
            c = self._param_id_list[i]
            targetPos = p.readUserDebugParameter(c)
            #p.setJointMotorControl2(self._robot_id, self._joint_ids[i], p.POSITION_CONTROL, targetPos, force=100 * 1.)
            #p.setJointMotorControl2(self._robot_id, self._joint_ids[i], p.TORQUE_CONTROL, force = targetPos)
            self._setMotorTorqueById(self._motor_id_list[i],targetPos)        
        
        """

        time.sleep(1./240.)

    def getMotorAngles(self):

        motor_angles = [
            p.getJointState(self._robot_id, motor_id)[0]
            for motor_id in self._motor_id_list
        ]
        motor_angles = np.multiply(motor_angles, self._motor_direction)
        return motor_angles

    def getMotorVelocities(self):

        motor_velocities = [
            p.getJointState(self._robot_id, motor_id)[1]
            for motor_id in self._motor_id_list
        ]
        motor_velocities = np.multiply(motor_velocities, self._motor_direction)
        return motor_velocities

    def setDesiredMotorAngleById(self, motor_id, desired_angle,kp=1,kd=10, max_force = 10):
        p.setJointMotorControl2(
        bodyIndex=self._robot_id,
        jointIndex=motor_id,
        controlMode=p.POSITION_CONTROL,
        targetPosition=desired_angle,
        positionGain=kp,
        velocityGain=kd,
        force=max_force)
    
    def setMotorTorqueById(self, motor_id, torque):
        p.setJointMotorControl2(
            bodyIndex=self._robot_id,
            jointIndex=motor_id,
            controlMode=p.TORQUE_CONTROL,
            force=torque)




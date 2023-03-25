'''
BipedModelクラスは、二足歩行ロボットの質量、寸法、関節制限などの物理的な特性や特徴を含んでいます。このクラスは biped_model.py ファイルで定義されています。

BipedModelクラスのパラメータは以下の通りです。

mass: 二足歩行ロボットの総質量（キログラム）。
height: ロボットの身長（メートル）。
leg_length: ロボットの足の長さ（メートル）。
foot_length: ロボットの足の長さ（メートル）。
hip_width: ロボットの股関節の幅 (メートル)。
max_joint_angle: ロボットの関節が届く最大の角度をラジアン単位で指定します。
BipedModel クラスの関数は以下の通りです。

__init__(): BipedModel クラスのインスタンス変数を初期化するコンストラクタ・メソッドです。
get_mass()：ロボットの質量を返す。ロボットの質量を返す。
get_height()：ロボットの高さを返す。ロボットの高さを返します。
get_leg_length()：ロボットの脚の長さを返します。ロボットの足の長さを返す。
ロボットの足の長さを返す ロボットの足の長さを返す。
get_hip_width(): ロボットの股関節の幅を返します。
get_max_joint_angle()：ロボットの股関節の最大角度を返します。ロボットの関節が届く最大の角度を返します。
set_mass()：ロボットの質量を設定します。ロボットの質量を設定する。
set_height(): ロボットの高さを設定する
set_leg_length()：ロボットの脚の長さを設定する。ロボットの足の長さを設定する。
set_foot_length(): ロボットの足の長さを設定する。
set_hip_width(): ロボットの股関節の幅を設定します。
set_max_joint_angle()：ロボットの股関節の角度を設定します。ロボットの関節が届く最大の角度を設定します。
'''


import pybullet as p
from kdl_parser_py.urdf import *

class BipedModel:
    def __init__(self, urdf_file):
        (ok, self.tree) = treeFromFile(urdf_file)
        print(self.tree.getChain(self.tree.getNrOfSegments()-1).getName())

    @property
    def COG(self):
        total_mass = 0
        center_of_gravity = kdl.Vector()
        for seg_name in self.tree.getSegments():
            seg = self.tree.getSegment(seg_name)
            seg_chain = self.tree.getSegmentChain(seg_name)
            seg_mass = seg.getInertia().getMass()
            seg_cog = seg.getInertia().getCOG()
            seg_frame = seg_chain.getSegmentFrame(seg_chain.getNrOfSegments()-1)
            seg_cog_in_base = seg_frame.M * seg_cog + seg_frame.p
            center_of_gravity += seg_mass * seg_cog_in_base
            total_mass += seg_mass
        return self.center_of_gravity

"""
class Kinematics:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.fk_solver = ChainFkSolverPos_recursive(self.robot_model.chain)

    def forward_kinematics(self, joint_values):
        joint_positions = []
        for joint_name in self.robot_model.get_joint_names():
            joint_positions.append(joint_values[joint_name])

        joint_positions = tuple(joint_positions)
        fk_result = Frame()
        self.fk_solver.JntToCart(joint_positions, fk_result)

        return fk_result
"""


'''TODO
class JointModel:
    def __init__(self, body_id, joint_name, max_force, kp, kd):
        self._body_id = body_id
        self._joint_index = p.getJointInfo(self._body_id, joint_name)[0]
        self._max_force = max_force
        self._kp = kp
        self._kd = kd

    def set_torque(self, torque):
        p.setJointMotorControl2(bodyIndex=self._body_id,
                                jointIndex=self._joint_index,
                                controlMode=p.TORQUE_CONTROL,
                                force=torque)#, positionGain=0.0, velocityGain=0.0)

'''
class Model(BipedModel):
    def __init__(self, urdf_path):
        super().__init__(urdf_path)

    @property
    def robot_id(self):
        return robot_id
    
    @property
    def param_ids(self):
        return param_ids
    
    @property
    def joint_ids(self):
        return joint_ids
    
    def get_joint_states(self):
        pass

    def apply_joint_efforts(self, efforts):
        # apply torque to motors.
        pass

    def _calculate_inertia(self):
        # implementation details omitted
        pass

    def __update_position(self):
        # implementation details omitted
        pass
    


if __name__ == '__main__':
    import time 

    # Connect to the PyBullet physics server
    p.connect(p.GUI)  
    useFixedBase = True
    flags = p.URDF_INITIALIZE_SAT_FEATURES
    ground_pos = [0,0,-0.625]
    ground = p.loadURDF('../../config/world/ground.urdf', ground_pos, flags = flags, useFixedBase=useFixedBase)
    robot_pos = [0,0,0.5]
    robot_urdf_path = '../../config/models/simplebot_v10/model.urdf'
    robot_id = p.loadURDF(robot_urdf_path, robot_pos)
    p.setGravity(0, 0,-0.1)  # Set the gravity to -9.81 m/s^2 in the z-direction
    p.setTimeStep(1.0/240.0)   # Set the time step to 1/240 seconds
    p.setRealTimeSimulation(1)
    p.setPhysicsEngineParameter(numSolverIterations=10)
    p.changeDynamics(robot_id, -1, linearDamping=0, angularDamping=0)
        # Set the torque of the left motor to 1 N-m
    left_motor_id = 0  # Replace with the actual ID of the left motor joint
    torque = 1.0
    p.setJointMotorControl2(robot_id, left_motor_id, p.TORQUE_CONTROL, force=torque)
    joint_ids = []
    param_ids = []
    for j in range(p.getNumJoints(robot_id)):
        p.changeDynamics(robot_id, j, linearDamping=0, angularDamping=0)
        info = p.getJointInfo(robot_id, j)
        jointName = info[1]
        jointType = info[2]
        jointLowerLimit = info[8]
        jointUpperLimit = info[9]
        if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
            joint_ids.append(j)
            param_ids.append(p.addUserDebugParameter(jointName.decode("utf-8"), jointLowerLimit, jointUpperLimit, 0))
            print("joint ID : {}, Name : {}".format(j, jointName.decode("utf-8")))


    import sys
    import os
    # Add the path of the directory containing the module to the system path
    sys.path.append(os.path.abspath("../kinematics"))
    from kdl_kinematics import KDLKinematics
    model = BipedModel(robot_urdf_path)
    

    while p.isConnected():
        p.stepSimulation()
        pos, orn = p.getBasePositionAndOrientation(robot_id)  # Get the position and orientation of the robot's base link

        for i in range(len(param_ids)):
            c = param_ids[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(robot_id, joint_ids[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
        
        '''        
        print("*************************")
        print("Robot position:",  )
        print("Robot orientation:", orn)
        '''
        time.sleep(1./240.)
    p.disconnect()

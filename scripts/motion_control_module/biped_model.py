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

class BipedModel:
    def __init__(self, robot_id):

        

        #pypullet
        self._robot_id = robot_id
        self.__joint_ids = []
        self.__param_ids = []
        for j in range(p.getNumJoints(self._robot_id)):
            p.changeDynamics(self._robot_id, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self._robot_id, j)
            #print(info)
            jointName = info[1]
            jointType = info[2]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.__joint_ids.append(j)
                self.__param_ids.append(p.addUserDebugParameter(jointName.decode("utf-8"), jointLowerLimit, jointUpperLimit, 0))
                print(jointName.decode("utf-8"))

    @property
    def robot_id(self):
        return self._robot_id
    
    @property
    def param_ids(self):
        return self.__param_ids
    
    @property
    def joint_ids(self):
        return self.__joint_ids
    
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

'''TODO
    @property
    def mass(self):
        return self._mass

    @property
    def height(self):
        return self._height

    @property
    def foot_length(self):
        return self._foot_length

    @property
    def foot_width(self):
        return self._foot_width

    @property
    def joint_limits(self):
        return self._joint_limits
'''

import pybullet_data

if __name__ == '__main__':

    p.connect(p.GUI)  # Connect to the PyBullet physics server

    print("This will always execute")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    useFixedBase = True
    flags = p.URDF_INITIALIZE_SAT_FEATURES

    ground_pos = [0,0,-0.625]
    ground = p.loadURDF("../../world/ground.urdf", ground_pos, flags = flags, useFixedBase=useFixedBase)
    robot_pos = [0,0,0.5]
    robot_id = p.loadURDF("../../models/simplebot_v10/model.urdf", robot_pos)

    model = BipedModel(robot_id)
    p.setPhysicsEngineParameter(numSolverIterations=10)
    p.changeDynamics(model.robot_id, -1, linearDamping=0, angularDamping=0)


    p.setGravity(0, 0,-0.1)  # Set the gravity to -9.81 m/s^2 in the z-direction
    p.setTimeStep(1.0/240.0)   # Set the time step to 1/240 seconds
    p.setRealTimeSimulation(1)
        
    while p.isConnected():
        p.stepSimulation()
        pos, orn = p.getBasePositionAndOrientation(model.robot_id)  # Get the position and orientation of the robot's base link
        for i in range(len(model.param_ids)):
            c = model.param_ids[i]
            targetPos = p.readUserDebugParameter(c)
            p.setJointMotorControl2(model.robot_id, model.joint_ids[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
        '''        print("*************************")
        print("Robot position:",  )
        print("Robot orientation:", orn)'''
    


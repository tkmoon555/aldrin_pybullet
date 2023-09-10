#! /usr/bin/env python

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
import urdf_parser_py.urdf as urdf
from .pykdl_utils.kdl_parser import *
import PyKDL as kdl
from urdf_parser_py.urdf import Robot

class BaseUrdf2KdlModel:
    base_link = None
    left_leg_link = None
    right_leg_link = None
    urdf_path = None
    def __init__(self):
        with open(self.urdf_path) as urdf_path:
            #rrrr = urdf.URDF.from_xml_string(urdf_path.read())
            robot = Robot.from_xml_string(urdf_path.read())
            self.tree = kdl_tree_from_urdf_model(robot)
        num_non_fixed_joints = 0
        print(robot.joint_map.keys())
        print(robot.link_map.keys())



        #print(self.tree)
        
        self._left_leg_chain = kdl.Chain()
        self._left_leg_chain = self.tree.getChain(self.base_link, self.left_leg_link)
        self._right_leg_chain = kdl.Chain()
        self._right_leg_chain = self.tree.getChain(self.base_link, self.right_leg_link)

    @property
    def left_leg_chain(self):
        return self._left_leg_chain
    @property
    def nrof_left_leg_joints(self):
        return self._left_leg_chain.getNrOfJoints()
    
    @property
    def right_leg_chain(self):
        return self._right_leg_chain
    
class BlackBirdModel(BaseUrdf2KdlModel):
    base_link = "torso"
    left_leg_link = "l_foot"
    right_leg_link = "r_foot"
    urdf_path = "../config/models/blackbird_urdf/urdf/blackbird.urdf"
    def __init__(self,name):
        super().__init__()
        self.name = name

        #child_segments = self.tree.getSegment(0)



class BipedKdlModel:
    base_link = "body_v1:1"
    left_leg_link = "arm_v4:3"
    right_leg_link = "arm_v4:4"
    urdf_file = '../../config/models/simplebot_v10/model.urdf'
    def __init__(self):

        with open(self.urdf_file) as urdf_file:
            self.tree = kdl_tree_from_urdf_model(urdf.URDF.from_xml_string(urdf_file.read()))
        print(self.tree)

        self._left_leg_chain = kdl.Chain()
        self.__set_tip_of_left_leg_chain()
        
        self._right_leg_chain = kdl.Chain()
        self.__set_tip_of_right_leg_chain()

        self._COG = kdl.Vector(0,0,0)

    @property
    def left_leg_chain(self):
        return self._left_leg_chain
    @property
    def right_leg_chain(self):
        return self._right_leg_chain
    
    def __set_tip_of_left_leg_chain(self):
        self._left_leg_chain = self.tree.getChain(self.base_link, self.left_leg_link)
        #segment = kdl.Segment("left_leg_tip",kdl.Joint(kdl.Joint.RotY),kdl.Frame(kdl.Vector(0,0,-0.1)))
        #segment = kdl.Segment(kdl.Joint(kdl.Joint("None")), kdl.Frame(kdl.Vector(0,0,-0.1)))     
        mass = 0.0856
        com = kdl.Vector(0.015, 0., -0.044)
        inertia = kdl.RotationalInertia(0., 0., 0., 0., 0., 0.)
        new_inertia = kdl.RigidBodyInertia(mass, com, inertia)
        segment = kdl.Segment("left_leg_tip",\
                              kdl.Joint(kdl.Vector(0,0,-0.1),kdl.Vector(1,0,0),kdl.Joint.RotAxis),\
                                kdl.Frame(kdl.Vector(0,0,-0.1)),\
                                    new_inertia)
        self._left_leg_chain.addSegment(segment)

    def __set_tip_of_right_leg_chain(self):
        self._right_leg_chain = self.tree.getChain(self.base_link, self.right_leg_link)
        mass = 0.0856
        com = kdl.Vector(0.015, 0., -0.044)
        inertia = kdl.RotationalInertia(0., 0., 0., 0., 0., 0.)
        new_inertia = kdl.RigidBodyInertia(mass, com, inertia)
        segment = kdl.Segment("right_leg_tip",\
                              kdl.Joint(kdl.Vector(0,0,-0.1),kdl.Vector(-1,0,0),kdl.Joint.RotAxis),\
                                kdl.Frame(kdl.Vector(0,0,-0.1)),\
                                    new_inertia)
        self._right_leg_chain.addSegment(segment)






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
class BipedModel(BipedKdlModel):
    def __init__(self):
        super().__init__()
    
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
    


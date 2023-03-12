'''
Biped Controller
Parameter

robot: 二足歩行ロボットの物理的特性を定義した BipedModel クラスのインスタンス。
kp: ロボットの動作の PID 制御に使用される比例ゲイン定数。
kd: ロボットの動作のPID制御に使用される微分ゲイン定数。

Function

BipedController クラスを、与えられたロボットのインスタンスと、オプションのパラメータ kp と kd で初期化します。
compute_next_action(self, state, target): 現在の状態とターゲットの状態から,ロボットの次のアクションを計算する．state パラメータにはロボットの関節角度、関節速度、足の位置などの情報が含まれ、target パラメータにはロボットの足の位置と速度の希望値が含まれる。この関数では、PIDコントローラを使用して、希望する足の位置と速度を実現するために必要な関節角度と速度を決定します。
get_joint_angles(self, state, target): 希望する足の位置と速度を実現するために必要な、次の関節角度のセットを計算する。
get_joint_velocities(self, state, target): 望みの足の位置と速度を得るために必要な、次のジョイントの速度のセットを計算します。
clip_joint_angles(self, joint_angles): 関節の角度をクリップして、ロボットの関節の物理的な限界内にあることを確認します。
clip_joint_velocities(self, joint_velocities): 関節の速度をクリップして、ロボットの関節の物理的限界内にあることを確認します。



全体として、BipedController クラスは、望ましい足の位置と速度を実現するために必要な関節角度と速度を計算し、二足歩行ロボットの動きを制御する重要な役割を担っています。比例ゲイン定数と微分ゲイン定数を持つPIDコントローラを使用することで、ロボットの動きにロバストな制御系を実現している。
'''
import numpy as np

from biped_model import BipedModel
from biped_kinematics import BipedKinematics

class BipedController(BipedKinematics):
    def __init__(self, controller_type='position', kp=0.5, kd=0.1, ki=0.01, max_effort=5.0):
        super().__init__()
        self.controller_type = controller_type
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.max_effort = max_effort
        
    def update(self, dt):
        # Read sensor data
        accelerations = self.sensors.get_accelerations()
        gyro_rates = self.sensors.get_gyro_rates()
        
        # Calculate desired joint angles for walking
        foot_positions = self.kinematics.get_foot_positions(self.stride_length, self.step_height, self.walk_speed, dt)
        joint_angles = self.kinematics.get_joint_angles(foot_positions)
        
    # Update PID controllers for joint position and velocity
    def _compute_effort(self, current_pos, current_vel, target_pos, target_vel):
        error = target_pos - current_pos
        error_dot = target_vel - current_vel
        effort = self.kp * error + self.kd * error_dot + self.ki * np.sum(error)
        return np.clip(effort, -self.max_effort, self.max_effort)

    def set_controller_params(self, controller_type, kp=None, kd=None, ki=None, max_effort=None):
        self.controller_type = controller_type
        if kp:
            self.kp = kp
        if kd:
            self.kd = kd
        if ki:
            self.ki = ki
        if max_effort:
            self.max_effort = max_effort

    def _position_control(self, target_pos):
        current_pos, current_vel = self.get_joint_states()
        target_vel = np.zeros_like(target_pos)
        effort = self._compute_effort(current_pos, current_vel, target_pos, target_vel)
        self.apply_joint_effort(effort)

    def _velocity_control(self, target_vel):
        current_pos, current_vel = self.get_joint_states()
        target_pos = np.zeros_like(target_vel)
        effort = self._compute_effort(current_pos, current_vel, target_pos, target_vel)
        self.apply_joint_effort(effort)

    def control(self, target, controller_type=None):
        if controller_type:
            self.controller_type = controller_type

        if self.controller_type == 'position':
            self._position_control(target)
        elif self.controller_type == 'velocity':
            self._velocity_control(target)
        else:
            raise ValueError(f"Invalid controller type '{self.controller_type}'")
    '''TODO    
    def set_walk_parameters(self, stride_length: float, step_height: float, step_duration: float, walk_speed: float) -> None:
        self.stride_length = stride_length
        self.step_height = step_height
        self.step_duration = step_duration
        self.walk_speed = walk_speed
    '''
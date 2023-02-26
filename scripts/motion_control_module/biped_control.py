'''
Biped Controller
Parameter

robot: 二足歩行ロボットの物理的特性を定義した BipedModel クラスのインスタンス。
kp: ロボットの動作の PID 制御に使用される比例ゲイン定数。
kd: ロボットの動作のPID制御に使用される微分ゲイン定数。

Function

BipedController クラスを、与えられたロボットのインスタンスと、オプションのパラメータ kp と kd で初期化します。
compute_next_action(self, state, target): 現在の状態とターゲットの状態から，ロボットの次のアクションを計算する．state パラメータにはロボットの関節角度、関節速度、足の位置などの情報が含まれ、target パラメータにはロボットの足の位置と速度の希望値が含まれる。この関数では、PIDコントローラを使用して、希望する足の位置と速度を実現するために必要な関節角度と速度を決定します。
get_joint_angles(self, state, target): 希望する足の位置と速度を実現するために必要な、次の関節角度のセットを計算する。
get_joint_velocities(self, state, target): 望みの足の位置と速度を得るために必要な、次のジョイントの速度のセットを計算します。
clip_joint_angles(self, joint_angles): 関節の角度をクリップして、ロボットの関節の物理的な限界内にあることを確認します。
clip_joint_velocities(self, joint_velocities): 関節の速度をクリップして、ロボットの関節の物理的限界内にあることを確認します。



全体として、BipedController クラスは、望ましい足の位置と速度を実現するために必要な関節角度と速度を計算し、二足歩行ロボットの動きを制御する重要な役割を担っています。比例ゲイン定数と微分ゲイン定数を持つPIDコントローラを使用することで、ロボットの動きにロバストな制御系を実現している。
'''

'''
BipedKinematicsクラスは、二足歩行ロボットの関節角度や足の位置などの運動特性を計算するための関数です。このクラスはロボットの物理的特性を入力として受け取り、その特性に基づいて運動学的な値を計算します。以下、BipedKinematicsクラスのパラメータと関数です。


パラメータ

L1: 上肢セグメントの長さ。
L2: 下腿部の長さ
L3: 足のセグメントの長さです。
hip_offset: 股関節と上腿骨セグメントの重心との距離。
foot_offset: 足首のジョイントから足部の質量の中心までの距離。
joint_limits: ロボットの関節角度の制限値。各関節のタプルとして表現される。
関数は以下の通りです。

forward_kinematics(q): 3次元空間における足の位置と体の向きを返します。
inverse_kinematics(x, y, z, phi): 足と胴体の位置と姿勢を指定して、ロボットの逆運動学計算を行う。希望のポーズを実現するための関節角度のセットを返します。
get_support_polygon(q): 2次元空間におけるポリゴンの頂点を返す。
check_support_polygon(q, x, y): 指定された足の位置 x,y が、ロボットのサポートポリゴン内にあるかどうかを、関節角度のセット q を与えてチェックする。


'''

'''
class BipedKinematics(BipedModel):
    def __init__(self, mass, height, foot_length, foot_width):
        super().__init__(mass, height, foot_length, foot_width)

    def forward_kinematics(self, hip_angle, knee_angle, ankle_angle):
        # Implementation of forward kinematics equations
        # ...
        return foot_position, joint_positions

    def inverse_kinematics(self, foot_position):
        # Implementation of inverse kinematics equations
        # ...
        return hip_angle, knee_angle, ankle_angle
'''

import numpy as np

# Jacobian_matrix function should be changed to suit your purpose.
class BipedKinematics:
    def __init__(self, dh_params):

        self._dh_params = dh_params

        self.saved_params = {'trajectory':[]}
        self.add_error_param = lambda x: self.saved_params['trajectory'].append(x)

    @property
    def dh_params(self):
        return self._dh_params.get_dh_params()


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    angles=np.array([0., 0., 0.])
    L=np.array([0.5, 0.5, 0.5])
    arm_dh_param = DHParams(angles, L)
    arm = BipedKinematics(arm_dh_param)

    # Define the target position
    p_target = np.array([0.3, 0.5, 0.3])

    # Create figure
    fig = plt.figure(figsize=(10,6))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax3 = fig.add_subplot(2, 2, 3)

    # graph1
    T_list = arm.forward_kinematics(angles)
    for i in range(len(T_list)-1):
        p1 = T_list[i][:3, 3]
        p2 = T_list[i+1][:3, 3]
        ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
        ax1.plot(p1[0], p1[1], p1[2], 'b',marker='o')
    ax1.plot(p_target[0],p_target[1],p_target[2], 'r',marker='o')

    # graph2
    angles, error_list = arm.inverse_kinematics(p_target,1000,1e-6,True)
    T_list = arm.forward_kinematics(angles)
    for i in range(len(T_list)-1):
        p1 = T_list[i][:3, 3]
        p2 = T_list[i+1][:3, 3]
        ax2.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
        ax2.plot(p1[0], p1[1], p1[2], 'b',marker='o')
    ax2.plot(p_target[0],p_target[1],p_target[2], 'r',marker='o')

    # graph3
    ax3.plot(error_list['trajectory'])

    # Setting glaph
    ax1.set_title('Initial')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-1, 1)
    ax1.set_ylim(-1, 1)
    ax1.set_zlim(0, 2)
    ax1.grid()

    ax2.set_title('Calculated')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-1, 1)
    ax2.set_ylim(-1, 1)
    ax2.set_zlim(0, 2)
    ax2.grid()

    ax3.set_title('Distance to the target per loop of ik function')
    ax3.grid()
    ax3.set_xlabel('iteration')

    plt.show()

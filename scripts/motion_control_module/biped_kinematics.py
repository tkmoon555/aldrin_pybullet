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

def ik_solver(dh_params, target_pose):
    # extract target position and orientation
    target_position = target_pose[:3, 3]
    target_orientation = target_pose[:3, :3]

    # initialize joint angles
    num_joints = len(dh_params)
    thetas = np.zeros(num_joints)

    # calculate homogeneous transformation matrices
    T_0_n = np.eye(4)
    for i, params in enumerate(dh_params):
        a, alpha, d, theta = params
        if i == 0:
            T_i_iminus1 = np.array([
                [np.cos(theta), 0, np.sin(theta), a],
                [np.sin(theta), 0, -np.cos(theta), 0],
                [0, 1, 0, d],
                [0, 0, 0, 1]
            ])
        else:
            T_i_iminus1 = np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])
        T_0_n = T_0_n @ T_i_iminus1

    # calculate inverse kinematics
    wrist_position = T_0_n[:3, 3]
    wrist_orientation = T_0_n[:3, :3]
    target_position_0 = np.linalg.inv(T_0_n) @ np.append(target_position, [1])
    target_position_0 = target_position_0[:3]
    target_orientation_0 = target_orientation @ wrist_orientation.T
    thetas[0] = np.arctan2(target_position_0[1], target_position_0[0])
    p = target_position_0 - dh_params[-1][0] * target_orientation_0[:, 2]
    m = np.sqrt(p[0]**2 + p[1]**2)
    n = p[2] - dh_params[0][2]
    a = dh_params[1][0]
    b = dh_params[2][0]
    c = np.sqrt(a**2 + b**2)
    alpha = np.arctan2(n, m)
    beta = np.arccos((a**2 + c**2 - b**2) / (2 * a * c))
    gamma = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))
    thetas[1] = np.pi/2 - alpha - beta
    thetas[2] = np.pi - gamma

    # calculate Jacobian matrix
    J = np.zeros((6, num_joints))
    z_axes = T_0_n[:3, 2]
    o_i = np.zeros(3)
    for i in range(num_joints):
        a_i, alpha_i, d_i, theta_i = dh_params[i]
        if i == 0:
            o_i = np.array([0, 0, d_i])
        elif i == 1:
            o_i = np.array([a_i, 0, d_i])
        elif i == 2:
            o_i = np.array([a_i, 0, d_i])
        else:
            o_i = np.array([a_i, 0, d_i])
        J[:3, i] = np.cross(z_axes, wrist_position - o_i)
        J[3:, i] = z_axes
    return thetas
def forward_kinematics(dh_params):
    # initialize homogeneous transformation matrix
    T = np.eye(4)

    # calculate homogeneous transformation matrix
    for i, params in enumerate(dh_params):
        a, alpha, d, theta = params
        T_i_iminus1 = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])
        T = T @ T_i_iminus1

    # calculate end-effector position and orientation
    position = T[:3, 3]
    orientation = T[:3, :3]

    return position, orientation

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_arm(dh_params, joint_angles):
    # Calculate the homogeneous transformation matrices for each link
    num_joints = len(dh_params)
    T_i_iminus1 = [np.eye(4)]
    for i, params in enumerate(dh_params):
        a, alpha, d, theta = params
        T_i_iminus1_i = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        T_i_iminus1.append(T_i_iminus1[-1] @ T_i_iminus1_i)

    # Calculate the position of the end effector
    end_effector_position = T_i_iminus1[-1][:3, 3]

    # Set up 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(-0.5, 0.5)
    ax.set_ylim3d(-0.5, 0.5)
    ax.set_zlim3d(0, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Plot each link as a line segment
    for i in range(num_joints):
        link_start = T_i_iminus1[i][:3, 3]
        link_end = T_i_iminus1[i+1][:3, 3]
        ax.plot([link_start[0], link_end[0]], [link_start[1], link_end[1]], [link_start[2], link_end[2]])

    # Plot the end effector as a sphere
    ax.scatter(end_effector_position[0], end_effector_position[1], end_effector_position[2], marker='o', s=50)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    # define DH parameters for the robot arm
    #a, alpha, d, theta
    dh_params = [
        [0.1, 0, 0, 0],
        [0.1, 0, 0, 0],
        [0.1, 0, 0, 0]
    ]

    target_position = np.array([0.25, 0, 0.])
    target_orientation = np.eye(3)
    target_pose = np.eye(4)
    target_pose[:3, :3] = target_orientation
    target_pose[:3, 3] = target_position

    # solve inverse kinematics
    thetas = ik_solver(dh_params, target_pose)

    dh_params = np.array(dh_params)
    dh_params[:,2] = dh_params[:,2] + thetas
    print(dh_params)
    # calculate forward kinematics to get end-effector position
    end_effector_position ,_ = forward_kinematics(dh_params.tolist())

    print(thetas)
    print(end_effector_position[0], end_effector_position[1], end_effector_position[2])
    
    visualize_arm(dh_params, thetas)
    '''    # plot 3D graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(end_effector_position[0], end_effector_position[1], end_effector_position[2], c='r', marker='o')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()'''

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
    
    
    def forward_kinematics(self, angles):
        self._dh_params.update_angle(angles)
        n = len(self.dh_params)
        T = np.eye(4)
        T_list = [T]
        for i in range(n):
            Ti = self._dh_to_matrix(self.dh_params[i])
            T = T @ Ti
            T_list.append(T)
        return T_list
    
    def inverse_kinematics(self, p_target, 
                           max_iterations = 1000, 
                           tolerance = 1e-6,
                           save_param_request = False):
        sr_gain = 0.1
        sr_I = sr_gain * np.eye(len(self.dh_params))
        angles = self._dh_params.get_joint_angles()
        for i in range(max_iterations):
            # Calculate the current position of the end-effector
            p_current = self.forward_kinematics(angles)[-1][:3, 3]
            # Calculate the error vector and check for convergence
            error = p_target - p_current
            # Distance to Target
            error_abs = np.linalg.norm(error)
            # Save error  
            if save_param_request:
                self.add_error_param(error_abs)
            # If the distance from the target is small enough, loop ends
            if error_abs < tolerance:
                break
            # Calculate the Jacobian matrix
            J = self._jacobi_matrix()
            # Calculate the inverse of the Jacobian matrix
            J_inv = np.linalg.pinv(J + sr_I )
            # Calculate the joint velocities
            v = J_inv @ error
            # Update the joint angles
            angles +=  v
            # Update the DH parameters
            self._dh_params.update_angle(angles)        
        if save_param_request:
            params = self.saved_params
            self.saved_params = {'trajectory':[]}
            return angles, params
        else:
            return angles

    def _dh_to_matrix(self,dh_params):
        #Convert DH parameters to a homogeneous transformation matrix.
        # 0_T_1 = Rx0 * Tx0 * Rz1 * Tz0
        a, alpha, d, theta = dh_params
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        Ti = np.array([
            [cos_theta, -sin_theta, 0, a],
            [cos_alpha*sin_theta, cos_alpha*cos_theta, -sin_alpha, -d * sin_alpha],
            [sin_alpha*sin_theta, sin_alpha*cos_theta, cos_alpha, d*cos_alpha],
            [0, 0, 0, 1]
        ])
        return Ti
    def _jacobi_matrix(self):
        # This function should be changed to suit your purpose.
        return self._dh_params.get_jacobian()


class DHParams:
    def __init__(self, angles, L):
        self.__angles =  angles
        self.lengths = L
        self.update_angle(angles)
        
    def get_dh_params(self):
        return self.__dh_params
    def get_jacobian(self):
        return self.__jacobian
    def get_joint_angles(self):
        return self.__angles
    def update_angle(self, angles):
        q1, q2, q3 = self.__angles =  angles
        l1, l2, l3 = self.lengths
        # define DH parameters for the robot arm
        # a, alpha, d, theta
        self.__dh_params = np.array([
            [0., 0., l1, q1],
            [0., q2, l2, 0.],
            [0., q3, l3, 0.]
        ])
        self.__jacobian = np.array([
            [l2*np.sin(q2)*np.cos(q1) + l3*np.sin(q2)*np.cos(q1)*np.cos(q3) + l3*np.sin(q3)*np.cos(q1)*np.cos(q2), l2*np.sin(q1)*np.cos(q2) - l3*np.sin(q1)*np.sin(q2)*np.sin(q3) + l3*np.sin(q1)*np.cos(q2)*np.cos(q3), -l3*np.sin(q1)*np.sin(q2)*np.sin(q3) + l3*np.sin(q1)*np.cos(q2)*np.cos(q3)], 
            [l2*np.sin(q1)*np.sin(q2) + l3*np.sin(q1)*np.sin(q2)*np.cos(q3) + l3*np.sin(q1)*np.sin(q3)*np.cos(q2), -l2*np.cos(q1)*np.cos(q2) + l3*np.sin(q2)*np.sin(q3)*np.cos(q1) - l3*np.cos(q1)*np.cos(q2)*np.cos(q3), l3*np.sin(q2)*np.sin(q3)*np.cos(q1) - l3*np.cos(q1)*np.cos(q2)*np.cos(q3)], 
            [0., -l2*np.sin(q2) - l3*np.sin(q2)*np.cos(q3) - l3*np.sin(q3)*np.cos(q2), -l3*np.sin(q2)*np.cos(q3) - l3*np.sin(q3)*np.cos(q2)]
        ])
        return self.__dh_params

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

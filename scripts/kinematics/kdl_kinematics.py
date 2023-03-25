#https://github.com/RethinkRobotics/baxter_pykdl/blob/master/src/baxter_kdl/kdl_kinematics.py
import PyKDL as kdl
import numpy as np
#from kdl_parser_py.urdf import *
class KDLKinematics:
    def __init__(self, urdf, base_link, end_link):
        (ok, tree) = treeFromFile(urdf)
        print(tree)
        self.chain = tree.getChain(base_link, end_link)
        self._kdl_fk        = kdl.ChainFkSolverPos_recursive(self.chain)
        self._kdl_ik        = kdl.ChainIkSolverPos_LMA(self.chain)
        self._kdl_angles     = kdl.JntArray(self.chain.getNrOfJoints())
        self.num_joints    = self.chain.getNrOfJoints()

        self._kdl_jac = kdl.ChainJntToJacSolver(self.chain)
        self._kdl_dyn = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())

    def fk(self, joint_angles, link_num = -1):
        for i,q in enumerate(joint_angles):
            self._kdl_angles[i]=q

        final_frame = kdl.Frame()

        if self._kdl_fk.JntToCart(self._kdl_angles, final_frame, link_num) < 0: print('\033[31m' + 'Please check link_num of KDLKinematics.fk' +'\033[0m')
        
        H = np.eye(4) #a homogeneous transformation matrix
        H[:3,:3] = np.array([[final_frame.M[0,0], final_frame.M[0,1], final_frame.M[0,2]],
                            [final_frame.M[1,0], final_frame.M[1,1], final_frame.M[1,2]],
                            [final_frame.M[2,0], final_frame.M[2,1], final_frame.M[2,2]]],
                            dtype=np.float32)
        H[:3, 3] = np.array([final_frame.p[0],final_frame.p[1],final_frame.p[2]],
                            dtype=np.float32)
        return H
    
    def ik(self,position):
        desireFrame = kdl.Frame(kdl.Vector(
            position[0],
            position[1],
            position[2]
        ))

        q_out = kdl.JntArray(self.num_joints)
        self._kdl_ik.CartToJnt(self._kdl_angles, desireFrame, q_out)

        return np.array([q_out[i] for i in range(q_out.rows())])
    
    def jacobian(self,joint_angles):
        for i,q in enumerate(joint_angles):
            self._kdl_angles[i]=q
        jacobi_matrix= kdl.Jacobian(self.num_joints)
        self._kdl_jac.JntToJac(self._kdl_angles, jacobi_matrix)
        return np.array([jacobi_matrix[i, j] for i in range(jacobi_matrix.rows()) for j in range(jacobi_matrix.columns())], dtype=np.float32).reshape((jacobi_matrix.rows(), jacobi_matrix.columns()))



if __name__ == '__main__':
    import sys
    import os
    # Add the path of the directory containing the module to the system path
    sys.path.append(os.path.abspath("../utils"))
    from kdl_parser_py.urdf import *

    urdf_path = '../../config/models/simplebot_v10/model.urdf'

    kdl_kinematics = KDLKinematics(urdf_path,"body_v1:1", "arm_v4:3")

    #print(kdl_kinematics.fk([0,0],2))
    #print(kdl_kinematics.ik([0,0,0]))
    #print(kdl_kinematics.jacobian([np.pi,np.pi]))

    init_angles = [0,0]

    import matplotlib.pyplot as plt
    # Create figure
    fig = plt.figure(figsize=(10,6))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    ax3 = fig.add_subplot(2, 2, 3)

    T_list = [ kdl_kinematics.fk(init_angles,i) for i in range(kdl_kinematics.num_joints +1)]
    for i in range(len(T_list)-1):
        p1 = T_list[i][:3, 3]
        p2 = T_list[i+1][:3, 3]
        ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
        ax1.plot(p1[0], p1[1], p1[2], 'b',marker='o')
    
    target_pos = [0.02, 0.0, -0.07]
    angles = kdl_kinematics.ik(target_pos)
    T_list = [ kdl_kinematics.fk(angles,i) for i in range(kdl_kinematics.num_joints +1)]
    for i in range(len(T_list)-1):
        p1 = T_list[i][:3, 3]
        p2 = T_list[i+1][:3, 3]
        ax2.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
        ax2.plot(p1[0], p1[1], p1[2], 'b',marker='o')
    # Setting glaph
    ax1.set_title('Initial')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-0.05, 0.05)
    ax1.set_ylim(-0.05, 0.05)
    ax1.set_zlim(-0.1, 0.1)
    ax1.grid()

    ax2.set_title('Calculated')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-0.05, 0.05)
    ax2.set_ylim(-0.05, 0.05)
    ax2.set_zlim(-0.1, 0.1)
    ax2.grid()

    plt.show()
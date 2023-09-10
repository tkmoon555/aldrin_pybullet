#https://github.com/RethinkRobotics/baxter_pykdl/blob/master/src/baxter_kdl/kdl_kinematics.py
#https://github.com/cambel/ur3/blob/f26d85c61c196646524789cbe88384291ec438cf/ur_pykdl/src/ur_pykdl/ur_pykdl.py#L218

import PyKDL as kdl
import numpy as np

def frame_to_list(frame):
    pos = frame.p
    rot = kdl.Rotation(frame.M)
    rot = rot.GetQuaternion()
    return np.array([pos[0], pos[1], pos[2],
                     rot[0], rot[1], rot[2], rot[3]])


class KDLKinematics:
    def __init__(self, chain, q_min_list=None, q_max_list=None):
        self.chain = chain

        #print(last_segment.getFrameToTip())
        self._kdl_fk        = kdl.ChainFkSolverPos_recursive(self.chain)
        self._kdl_ik_v = kdl.ChainIkSolverVel_pinv(self.chain,0.001)


        self._num_jnts    = self.chain.getNrOfJoints()

        '''
        q_min = kdl.JntArray(len(self._num_jnts))
        q_max = kdl.JntArray(len(self._num_jnts))
        if q_min_list !=None or q_max_list !=None:
            for i,q in enumerate(q_min_list):
                q_min[i] = q
            for i,q in enumerate(q_max_list):
                q_max[i] = q
        else:
            for i in range(self._num_jnts):
                q_min[i] = -90 * np.pi/180
                q_max[i] =  90 * np.pi/180
        self._kdl_ik  = kdl.ChainIkSolverPos_NR_JL(self.chain, q_min, q_max, self._kdl_fk,self._kdl_ik_v,100,1e-5)
        '''
        if 1:
            eps_position = 1e-2

            maxiter = 100
            eps_joint = 1e-15
            self._kdl_ik        = kdl.ChainIkSolverPos_LMA(self.chain,eps_position,maxiter,eps_joint)
          

        
        self.num_segments = self.chain.getNrOfSegments()
        self._kdl_angles     = kdl.JntArray(self._num_jnts)


        self._kdl_jac = kdl.ChainJntToJacSolver(self.chain)
        self._kdl_dyn = kdl.ChainDynParam(self.chain, kdl.Vector.Zero())
    @property
    def num_jnts(self):
        return self._num_jnts
    def kdl_to_mat(self, data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat
    def forward_kinematics(self, joint_angles, link_num = -1, list_type=True):
        
        if self._num_jnts != len(joint_angles):
            print('\033[31m Please number of length joint_angles:{}, chain joints:{} KDLKinematics.fk \033[0m'.format(joint_angles, self.num_joints))
        for i,q in enumerate(joint_angles):
            self._kdl_angles[i]=q


        end_effector_frame = kdl.Frame()

        fk_status = self._kdl_fk.JntToCart(self._kdl_angles, end_effector_frame, link_num)


        if fk_status < 0:
            print('\033[31m' + 'Please check link_num of KDLKinematics.fk' +'\033[0m', fk_status)

        if list_type:
            return frame_to_list(end_effector_frame)
        else:
            H = np.eye(4) #a homogeneous transformation matrix
            H[:3,:3] = np.array([[end_effector_frame.M[0,0], end_effector_frame.M[0,1], end_effector_frame.M[0,2]],
                                [end_effector_frame.M[1,0], end_effector_frame.M[1,1], end_effector_frame.M[1,2]],
                                [end_effector_frame.M[2,0], end_effector_frame.M[2,1], end_effector_frame.M[2,2]]],
                                dtype=np.float32)
            H[:3, 3] = np.array([end_effector_frame.p[0],end_effector_frame.p[1],end_effector_frame.p[2]],
                                dtype=np.float32)
            return H

    
    def inverse_kinematics(self,position):
        desireFrame = kdl.Frame(kdl.Vector(
            position[0],
            position[1],
            position[2]
        ))

        q_out = kdl.JntArray(self.num_jnts)
        ik_status = self._kdl_ik.CartToJnt(self._kdl_angles, desireFrame, q_out)

        if ik_status >=0:
            #print(' _kdl_angles:{}, num_jnts:{}\033[0m'.format(self._kdl_angles,self.num_jnts))
            return np.array(list(q_out)) 
        else:
            print('\033[31m' + 'can not solve KDLKinematics.ik' +'\033[0m', ik_status)
            print('\033[31m comfirm joint_limit, _kdl_angles:{}, num_jnts:{}\033[0m'.format(self._kdl_angles,self.num_jnts))
            
            return np.array([None for i in range(q_out.rows())])

        

    def jacobian(self,joint_angles):
        for i,q in enumerate(joint_angles):
            self._kdl_angles[i]=q
        jacobi_frame= kdl.Jacobian(self.num_jnts)
        self._kdl_jac.JntToJac(self._kdl_angles, jacobi_matrix)
        #return np.array([jacobi_matrix[i, j] for i in range(jacobi_matrix.rows()) for j in range(jacobi_matrix.columns())], dtype=np.float32).reshape((jacobi_matrix.rows(), jacobi_matrix.columns()))
        return self.kdl_to_mat(jacobian)

    def jacobian_transpose(self, joint_values=None):
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self, joint_values=None):
        return np.linalg.pinv(self.jacobian(joint_values))

    def inertia(self, joint_values=None):
        inertia = kdl.JntSpaceInertiaMatrix(self._num_jnts)
        self._dyn_kdl.JntToMass(self.joints_to_kdl('positions', joint_values), inertia)
        return self.kdl_to_mat(inertia)

    def cart_inertia(self, joint_values=None):
        js_inertia = self.inertia(joint_values)
        jacobian = self.jacobian(joint_values)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)

if __name__ == '__main__':
    import sys
    import os
    # Add the path of the directory containing the module to the system path
    sys.path.append(os.path.abspath("../utils"))

    from biped_model import BipedKdlModel
    model = BipedKdlModel()


    dummy_angle = 0
    init_angles_list = [
        [0, 0.1,dummy_angle],
        [0, 0.1,dummy_angle]
    ]
    target_position_list = [
        np.array([0.02,0.14,-0.1]),
        np.array([-0.02,0.,-0.12])
    ]
    chain_list = [
        model.left_leg_chain,
        model.right_leg_chain
    ]


    import matplotlib.pyplot as plt
    # Create figure
    fig = plt.figure(figsize=(10,6))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    #ax3 = fig.add_subplot(2, 2, 3)

    for chain, init_angles, target_pos in zip( chain_list, init_angles_list, target_position_list):
        

        q_min_list = [-np.pi/2]*3
        q_max_list = [np.pi/2]*3
        kdl_kinematics = KDLKinematics(chain,q_min_list,q_max_list)
        
        print("#################################",kdl_kinematics.num_segments,"#################################" )


        T_list = [ kdl_kinematics.forward_kinematics(init_angles,i,list_type=False) for i in range(kdl_kinematics.num_segments+1)]

        for i in range(len(T_list)-1):
            p1 = T_list[i][:3, 3] 
            p2 = T_list[i+1][:3, 3]
            ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
            ax1.plot(p1[0], p1[1], p1[2], 'b',marker='o')
            print("norm",np.linalg.norm(p2-p1))
        
        print("p2",p2)

        angles = kdl_kinematics.inverse_kinematics(target_pos)
        print("angles",angles * 180./np.pi)

        T_list = [ kdl_kinematics.forward_kinematics(angles,i,list_type=False) for i in range(kdl_kinematics.num_segments+1)]
        for i in range(len(T_list)-1):
            p1 = T_list[i][:3, 3]
            p2 = T_list[i+1][:3, 3]
            ax2.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
            ax2.plot(p1[0], p1[1], p1[2], 'b',marker='o')
            print("norm",np.linalg.norm(p2-p1))
        print("p2",p2)
        print("error:",np.linalg.norm(target_pos-p2))


    # Setting glaph
    ax1.set_title('Initial')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-0.2, 0.2)
    ax1.set_ylim(-0.2, 0.2)
    ax1.set_zlim(-0.2, 0.2)
    ax1.grid()

    ax2.set_title('Calculated')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-0.2, 0.2)
    ax2.set_ylim(-0.2, 0.2)
    ax2.set_zlim(-0.2, 0.2)
    ax2.grid()

    plt.show()


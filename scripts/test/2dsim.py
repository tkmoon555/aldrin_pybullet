import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos

def dh_to_matrix(dh_param):
    """
    Convert DH parameters to a homogeneous transformation matrix.
    """
    a, alpha, d, theta = dh_param
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

def jacobi_matrix(thetas, lengths):
    l1 , l2, l3 = lengths
    q1, q2, q3 = thetas
    J = np.array([
        [1.0*l2*sin(q2)*cos(q1) + 1.0*l3*sin(q2)*cos(q1)*cos(q3) + 1.0*l3*sin(q3)*cos(q1)*cos(q2), 1.0*l2*sin(q1)*cos(q2) - 1.0*l3*sin(q1)*sin(q2)*sin(q3) + 1.0*l3*sin(q1)*cos(q2)*cos(q3), 0], 
        [1.0*l2*sin(q1)*sin(q2) + 1.0*l3*sin(q1)*sin(q2)*cos(q3) + 1.0*l3*sin(q1)*sin(q3)*cos(q2), -1.0*l2*cos(q1)*cos(q2) + 1.0*l3*sin(q2)*sin(q3)*cos(q1) - 1.0*l3*cos(q1)*cos(q2)*cos(q3), 0], 
        [0, -1.0*l2*sin(q2) - 1.0*l3*sin(q2)*cos(q3) - 1.0*l3*sin(q3)*cos(q2), 1]
        ])
    return J


# Define the target position
p_target = np.array([0.3, 0.5, 0.3])

# Define the initial joint angles and DH parameters
#a, alpha, d, theta
theta = np.array([0., 0., 0.])
L = np.array([0.5,0.5, 0.5])
dh_params = np.array([
    [0.0, 0.0, L[0], theta[0]],
    [0.0, theta[1], L[1], 0.0],
    [0.0, theta[2], L[2], 0.0]
    ])

# Define the maximum number of iterations and the tolerance for convergence
max_iterations = 1000
tolerance = 1e-6

def forward_kinematics(dh_params):
    n = len(dh_params)
    T = np.eye(4)
    T_list = [T]
    for i in range(n):
        Ti = dh_to_matrix(dh_params[i])
        T = T @ Ti
        T_list.append(T)
    return T_list

error_list=[]
sr_gain = 0.1
sr_I = sr_gain * np.eye(3)
def inverse_kinematics(p_target, dh_params, max_iterations, tolerance):
    global theta
    for i in range(max_iterations):
        # Calculate the current position of the end-effector
        p_current = forward_kinematics(dh_params)[-1][:3, 3]
 
        # Calculate the error vector and check for convergence
        error = p_target - p_current

        # Save error         
        error_abs = np.linalg.norm(error)
        error_list.append(error_abs)

        # If the distance from the target is small enough, loop ends
        if error_abs < tolerance:
            print("get no error params")
            break
        
        # Calculate the Jacobian matrix
        J = jacobi_matrix(theta,L)

        # Calculate the inverse of the Jacobian matrix
        J_inv = np.linalg.pinv(J + sr_I)
        
        # Calculate the joint velocities
        v = np.dot(J_inv, error)

        # Update the joint angles
        theta += 0.1 * v
        
        # Update the DH parameters
        dh_params = np.array([
            [0.0, 0.0, L[0], theta[0]],
            [0.0, theta[1], L[1], 0.0],
            [0.0, theta[2], L[2], 0.0]
            ])

    return theta,dh_params

# Save inital dh param
start_dh_params = dh_params

# Perform inverse kinematics
theta,dh_params = inverse_kinematics(p_target, dh_params, max_iterations, tolerance)

# Print the final joint angles
print("Final joint angles:{}".format(theta))

# Create figure
fig = plt.figure(figsize=(10,6))
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax2 = fig.add_subplot(2, 2, 2)
ax3 = fig.add_subplot(2, 2, 3, projection='3d')

# graph1
T_list = forward_kinematics(start_dh_params)
for i in range(len(T_list)-1):
    p1 = T_list[i][:3, 3]
    p2 = T_list[i+1][:3, 3]
    ax1.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
    ax1.plot(p1[0], p1[1], p1[2], 'b',marker='o')
ax1.plot(p_target[0],p_target[1],p_target[2], 'r',marker='o')

# graph2
ax2.plot(error_list)

# graph3
T_list = forward_kinematics(dh_params)
for i in range(len(T_list)-1):
    p1 = T_list[i][:3, 3]
    p2 = T_list[i+1][:3, 3]
    ax3.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'b')
    ax3.plot(p1[0], p1[1], p1[2], 'b',marker='o')
ax3.plot(p_target[0],p_target[1],p_target[2], 'r',marker='o')

# Setting glaph
ax1.set_title('Initial')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_xlim(-1, 1)
ax1.set_ylim(-1, 1)
ax1.set_zlim(0, 2)
ax1.grid()

ax2.set_title('Distance to the target per loop of ik function')
ax2.grid()
ax2.set_xlabel('iteration')

ax3.set_title('Calculated')
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.set_xlim(-1, 1)
ax3.set_ylim(-1, 1)
ax3.set_zlim(0, 2)
ax3.grid()

plt.show()
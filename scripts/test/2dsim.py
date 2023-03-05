import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


l1 = 1
l2 = 2


L = np.array([[l1, l2]]).T

J2_1 = np.array([[1,1]]).T # 2x1 all-ones matrix
count  = 0

fig = plt.figure(figsize=(12,4))

ax1 = fig.add_subplot(2, 2, 1)
ax1.grid()
ax2 = fig.add_subplot(2, 2, 2)
ax2.grid()
ax3 = fig.add_subplot(2, 2, 3)
ax3.grid()
ax4 = fig.add_subplot(2, 2, 4)
ax4.grid()

cm = plt.get_cmap('Spectral')
ax2_xdata=[]
ax2_ydata=[]
ax3_xdata=[]
ax3_ydata=[]
ax4_xdata=[]
ax4_ydata=[]

def forward_Mat(Q):
    q1 = Q[0][0]
    q2 = Q[1][0]
    M = np.array([[np.cos(q1), np.cos((q1+q2))],
                  [np.sin(q1), np.sin((q1+q2))]])
    return M

def Jacobian(Q):
    q1 = Q[0,0]
    q2 = Q[1,0]
    J = np.array([[(-l1*np.sin(q1))-(l2*np.sin(q1 + q2)),-l2*np.sin(q1 + q2)],
                  [(l1*np.cos(q1))+(l2*np.cos(q1 + q2)), l2*np.cos(q1 + q2)]])
    return J

def forward_kinetic(Q):

    Mat = forward_Mat(Q)
    head = np.dot(Mat, L)  #2,1 先端の座標

    var = Mat*np.dot(J2_1,L.T) # アダマール積 A*B
    return var,head

def var2armp(var):
    zp = np.array([[0,0]]).T
#insert start point
    armp = np.insert(var , 0, zp.T, axis = 1)
#sums each point as if stair
    for i in range(armp.shape[1]): #列の和i = i + (i-1)
        if i==0:
            pass
        else:
            armp[:, i:i+1] = armp[:, i:i+1] + armp[:, i-1:i]
    return armp
def write_plot(armp):
    global count
    #ax1.set_xlim(-5, 5)
    ax1.set_aspect('equal')
    ax1.plot(armp[0], armp[1], color=cm(count), marker='.',markersize=1,
             markerfacecolor='blue',
             markeredgecolor='blue')

    count = (count+1)%360


def inverse_kinetic(target_P, now_Q):
    [target_x, target_z] = target_P
    [q1, q2] = now_Q #now Q

    sr_lambda = 1
    I = np.identity(2)

    for i in range(100):
        var,P = forward_kinetic(now_Q)
        Jacobi = Jacobian(now_Q) #2x2

        d_P = (target_P - P)
        jacobi_inv =np.linalg.inv( Jacobi + (sr_lambda*I) )
        d_Q = 0.1*np.dot(jacobi_inv, d_P)

        d_Q = d_Q

        now_Q = now_Q + d_Q


        #graph
        test(now_Q)
        [dx,dy]= np.ravel(d_P)
        [dq1,dq2]= np.ravel(d_Q)
        ax2_xdata.append(i+1)
        ax2_ydata.append(np.linalg.det(Jacobi))
        #ax2_ydata.append( np.sqrt(dx**2 + dy**2))
        ax3_xdata.append(i+1)
        ax3_ydata.append( dq1*180/np.pi)
        ax4_xdata.append(i+1)
        ax4_ydata.append( dq2*180/np.pi)
    return now_Q

def test(Q):
    var,head = forward_kinetic(Q)
    armp = var2armp(var)
    #print(head == armp[:,-1].reshape(2,1))
    write_plot(armp)


if __name__ == "__main__":
    q1 = 0 *(np.pi/180)
    q2 = 0  *(np.pi/180)
    Q = np.array([[q1,q2]]).T
    test(Q)

    target_P = np.array([[1,1]]).T
    Q = inverse_kinetic(target_P, Q)

    


    """
    d_Q = np.array([[85*(np.pi/180), 8*(np.pi/180)]])
    Jacobi = Jacobian(Q) #2x2
    Mat = forward_Mat(Q+d_Q)
    var = Mat*np.dot(J2_1,L) # アダマール積 A*B
    #d_var = Jacobi * np.dot(J2_1,d_q)# アダマール積 A*B
    #var = var + d_var
    d_head = np.dot(Jacobi, d_Q.T)
    head = head + d_head
    plt.plot(head[0], head[1], color='#30F010', marker='^',
             markerfacecolor='blue',
             markeredgecolor='blue')
    armp = var2armp(var)
    print(head == armp[:,-1].reshape(2,1))
    write_plot(armp)
    jacobi_inv =np.linalg.inv(Jacobi)
    d_P = np.array([[-1,-1]])
    d_P *=3
    print(np.dot(jacobi_inv,d_P.T))
    """



    ax2.plot(ax2_xdata, ax2_ydata, color=(0,0,0), marker='.',markersize=1,
             markerfacecolor='blue',
             markeredgecolor='blue')
    ax3.plot(ax3_xdata, ax3_ydata, color=(0,0,0), marker='.',markersize=1,
             markerfacecolor='blue',
             markeredgecolor='blue')
    ax4.plot(ax4_xdata, ax4_ydata, color=(0,0,0), marker='.',markersize=1,
             markerfacecolor='blue',
             markeredgecolor='blue')

    plt.tight_layout()
    plt.savefig("result"+".png")

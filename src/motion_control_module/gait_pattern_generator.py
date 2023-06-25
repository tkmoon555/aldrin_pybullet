import numpy as np
class GaitPatternGenerator:
    def __init__(self):
        self.walking_cycle_time = None
    
class GaitPattern:
    COG_Z = 0.8 #m
    SUPPORT_PHASE_TIME = 0.8#second
    def __init__(self, gravity, support_leg = 'left'):
        self.time_constant = np.sqrt(self.COG_Z/abs(gravity))

        self.gravity = gravity

        self._C = np.cosh(self.SUPPORT_PHASE_TIME/self.time_constant)
        self._S = np.sinh(self.SUPPORT_PHASE_TIME/self.time_constant)

        self.set_support_leg(support_leg)

        self._left_foot_position = [0.,0.,0.]
        self._right_foot_position = [0.,0.,0.]

        self._reference_landing_position = [None, None]


        self.__init_stock_params()
    # params for plot
    def __init_stock_params(self):
        self.__trace_cog_position = []
        
    def rotation_matrix(self,theta):
        return [[np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]]
        
    def get_gait_pattern(self, landing_position, step, next_step):
        ref_landing_position = landing_position + np.dot(self.rotation_matrix(step[2]),\
                                                         [step[0],
                                                          -self._which_leg*step[1] # <- init is right foot support leg
                                                          ])

        #final cog state at end of theSS support phase time
        ref_cog_position = np.dot(self.rotation_matrix(next_step[2]),\
            [next_step[0]/2,
             self._which_leg*next_step[1]/2])
        
        ref_cog_velocity = np.dot(self.rotation_matrix(next_step[2]),\
            [ref_cog_position[0]*(self._C + 1)/(self.time_constant*self._S),
             ref_cog_position[1]*(self._C - 1)/(self.time_constant*self._S)])
        
        self._reference_landing_position[0] = ref_landing_position[0]
        self._reference_landing_position[1] = ref_landing_position[1]
        return ref_landing_position, ref_cog_position, ref_cog_velocity 
    @property
    def left_foot_position(self):
        return self._left_foot_position.copy()
    @property
    def right_foot_position(self):
        return self._right_foot_position.copy()
    @property
    def reference_landing_position(self):
        return self._reference_landing_position.copy()
        
    def switch_support_leg(self):
        self._which_leg *= -1
    def set_support_leg(self, support_leg):
        if support_leg == 'left':
            self._which_leg = 1
        elif support_leg == 'right':
            self._which_leg = -1
        else:
            print('Error: No match support leg')
    def which_support_leg(self):
        if self._which_leg == 1:
            support_leg = 'left'
        elif self._which_leg == -1:
            support_leg = 'right'
        else:
            print('Error: No match support leg')
        return support_leg
    def generate_gait_pattern(self, landing_position, cog_position, cog_velocity, step, next_step, dt=1e-3, trace= False):
        self.__init_stock_params()

        landing_position = np.array(landing_position)
        cog_position = np.array(cog_position)
        cog_velocity = np.array(cog_velocity)

        for i in range(int(self.SUPPORT_PHASE_TIME/dt)):
            cog_accelration = self._inverted_pendulum_equation(cog_position, landing_position)
            cog_velocity += cog_accelration * dt
            cog_position += cog_velocity * dt
            #To plot for centor of gravity
            if trace == True: self.__stock_cog(cog_position) 

        #reference
        ref_landing_position, ref_cog_position, ref_cog_velocity = self.get_gait_pattern(landing_position, step, next_step)

        desired_cog_position = ref_landing_position + ref_cog_position
        desired_cog_velocity = ref_cog_velocity

        # Evaluation gain
        a = 10.0
        b = 1.0

        # Revised landing position
        D = (a*((self._C -1)**2)) + (b*((self._S/self.time_constant)**2))
        landing_position = (-a*((self._C-1)/D))*(desired_cog_position - (self._C*cog_position) - (self.time_constant*self._S*cog_velocity))\
            - ((b*self._S)/(self.time_constant*D))*(desired_cog_velocity - (self._S/self.time_constant*cog_position) - (self._C*cog_velocity))
        
        if self._which_leg == 1:
            self._right_foot_position = landing_position
        else:
            self._left_foot_position = landing_position
        
        return landing_position, cog_position, cog_velocity

    def _inverted_pendulum_equation(self, cog_position, landing_position):
        # Equation of an inverted pendulum
        return (self.gravity/self.COG_Z)*(cog_position-landing_position) # m/s^2

    def get_stock_params(self):
        return self.__trace_cog_position
    
    def __stock_cog(self, cog_position):
        self.__trace_cog_position.append(cog_position.copy())





if __name__ == '__main__':
    import matplotlib.pyplot as plt  
    from matplotlib import animation
    #draw
    fig = plt.figure(figsize=(10, 20), facecolor='lightblue')

    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2)

    gp = GaitPattern(9.8)
    
    step_params = 10*[[0.,0.25,0]]\
    +[ [0.15,0.25,20/180*np.pi],
      [0.15,0.25,40/180*np.pi],
      [0.15,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
      [0.25,0.25,40/180*np.pi],
       ]

    landing_position = [0.,0.]
    cog_position = [0.,0.]
    cog_velocity = [0.15,0.1]

    land_pos= []
    right_land_pos = []
    left_land_pos = []
    trace_cog_pos = []
    reference_landing_pos = []

    for i in range(len(step_params)-1):

        landing_position,cog_position, cog_velocity = gp.generate_gait_pattern(landing_position,cog_position, cog_velocity,\
                                                                                 step_params[i], step_params[i+1],trace=True)
        trace_cog_position= gp.get_stock_params()
        reference_landing_position = gp.reference_landing_position
        right_foot_pos = gp.right_foot_position
        left_foot_pos = gp.left_foot_position

        land_pos.append(landing_position)
        trace_cog_pos.extend(trace_cog_position)
        reference_landing_pos.append(reference_landing_position)
        right_land_pos.append(right_foot_pos)
        left_land_pos.append(left_foot_pos)

        gp.switch_support_leg()
    


    ax1.plot([row[0] for row in land_pos], [row[1] for row in land_pos], 'bo')
    ax1.plot([row[0] for row in right_land_pos], [row[1] for row in right_land_pos], 'go')
    ax1.plot([row[0] for row in left_land_pos], [row[1] for row in left_land_pos], 'yo')

    ax1.plot([row[0] for row in trace_cog_pos],[row[1] for row in trace_cog_pos],'ro',markersize=1)
    ax1.plot([row[0] for row in reference_landing_pos],[row[1] for row in reference_landing_pos],'r*',markersize=10)
    
    ax1.grid(color='r', linestyle='dotted', linewidth=1)

    #ax1.set_xlim(-2., 5.)
    #ax1.set_ylim(-2., 5.)
    ims = []  #ここに1ステップごとのグラフを格納
    for i in range(len(land_pos)):
        p = ax2.plot([row[0] for row in land_pos[:i]], [row[1] for row in land_pos[:i]], color = 'darkblue', marker = 'o', markersize = 10)+\
              ax2.plot([row[0] for row in reference_landing_pos[:i]],[row[1] for row in reference_landing_pos[:i]],'r*',markersize=10)

        #ax2.set_xlim(-2., 5.)
        #ax2.set_ylim(-2., 5.)
        ims.append(p)

    ani = animation.ArtistAnimation(fig, ims, interval=1000)  #ArtistAnimationでアニメーションを作成する。

    plt.show()
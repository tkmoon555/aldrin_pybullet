import numpy as np
class GaitPatternGenerator:
    def __init__(self):
        self.walking_cycle_time = None
    



class GaitPattern:
    COG_Z = 1 #m
    SUPPORT_PHASE_TIME = 2 #second
    def __init__(self, gravity):
        self.time_constant = np.sqrt(self.COG_Z/abs(gravity))
        self.support_phase_time = self.SUPPORT_PHASE_TIME

        self.gravity = gravity


        self.C = np.cosh(self.support_phase_time/self.time_constant)
        self.S = np.sinh(self.support_phase_time/self.time_constant)

        self.which_leg = -1

        self.landing_position_x = 0.
        self.landing_position_y = 0.

        self.walking_cycle_time = None
        self.step_x = []
        self.step_y = []
    def __iter__(self):
        self._count = 0
        return self
    def which_leg(self):
        self.which_leg *= self.which_leg

    def get_landing_position(self, step, next_step):
        self.which_leg()

        self.landing_position_x += step[0]
        self.landing_position_y += -self.which_leg*step[1]
    
        gaitpattern_x = next_step[0]/2
        gaitpattern_y = self.which_leg*next_step[1]/2

        velocity_x = (self.C + 1)/(self.time_constant*self.S)*gaitpattern_x
        velocity_y = (self.C - 1)/(self.time_constant*self.S)*gaitpattern_y

        return velocity_x, velocity_y
    
    def inverted_pendulum_equation(self, cog_position, landing_position):
        #Equation of an inverted pendulum
        acceleration_x = (self.gravity/self.COG_Z)*(self.position[0])
        acceleration_y = (self.gravity/self.COG_Z)*(self.position[1])

        

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    gp = GaitPattern(9.8)
    
    """
    fig = plt.figure(figsize=(10,6))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    x = np.linspace(0,0.1,5)
    """
    step_params = [
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1],
        [0.1,0.1]

    ]
    for i in len(step_params)-1:
        print(gp.get_landing_position(step_params[i], step_params[i+1]))



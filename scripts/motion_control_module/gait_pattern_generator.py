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

        self.cog_position = np.array([0.,0])
        self.cog_velocity = np.array([0.,0])
        self.landing_position = np.array([0,0])

    def __iter__(self):
        self._count = 0
        return self
    def swich_leg(self):
        self.which_leg *= self.which_leg

    def get_gait_pattern(self, step, next_step):
        self.swich_leg()
        next_landing_position = np.array([
            self.landing_position_x + step[0],
            self.landing_position_y - self.which_leg*step[1]
            ])
        next_cog_position = np.array([
            next_step[0]/2,
            self.which_leg*next_step[1]/2
            ])
        next_cog_velocity = np.array([
            (self.C + 1)/(self.time_constant*self.S)*next_cog_position[0],
            (self.C - 1)/(self.time_constant*self.S)*next_cog_position[1]
            ])
        return next_landing_position, next_cog_position, next_cog_velocity 
    
    def generate_gait_pattern(self, step, next_step, dt=1e-5):
        for i in range(int(self.support_phase_time/dt)):
            cog_accelration = self.inverted_pendulum_equation(self, self.cog_position, self.landing_position)
            self.cog_velocity += cog_accelration * dt
            self.cog_position += self.cog_velocity * dt

        next_landing_position, next_cog_position, next_cog_velocity  = self.get_gait_pattern(step, next_step)

        desired_cog_position = np.array([
            next_landing_position[0] + next_cog_position[0],
            next_landing_position[1] + next_cog_position[1]
            ])
        desired_cog_velocity = np.array([
            next_cog_velocity[0], 
            next_cog_velocity[1]
            ])
        
        # Evaluation gain
        a = 1.0
        b = 1.0
        
        # Revised landing position
        D = (a*((self.C -1)**2)) + b*((self.S/self.time_constant)**2)
        self.landing_position = (-a*((self.C-1)/D))*(desired_cog_position - (self.C*self.cog_position) - self.time_constant*self.S*self.cog_velocity)\
            - ((b*self.S)/(self.time_constant*D))*(desired_cog_velocity - (self.S/self.time_constant*self.cog_position) - (self.C*self.cog_velocity))
        
    def inverted_pendulum_equation(self, cog_position, landing_position):
        # Equation of an inverted pendulum
        return (self.gravity/self.COG_Z)*(cog_position-landing_position) # m/s^2
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
        pass



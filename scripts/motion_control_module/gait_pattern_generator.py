import numpy as np
class GaitPatternGenerator:
    def __init__(self):
        self.walking_cycle_time = None
    
class GaitPattern:
    COG_Z = 0.8 #m
    SUPPORT_PHASE_TIME = 0.8 #second
    def __init__(self, gravity):
        self.time_constant = np.sqrt(self.COG_Z/abs(gravity))
        self.support_phase_time = self.SUPPORT_PHASE_TIME

        self.gravity = gravity


        self.C = np.cosh(self.support_phase_time/self.time_constant)
        self.S = np.sinh(self.support_phase_time/self.time_constant)

        self.which_leg = -1


        self.walking_cycle_time = None
        self.step_x = []
        self.step_y = []

        self.cog_position = np.array([0.,0])
        self.cog_velocity = np.array([0.,0])
        

    def __iter__(self):
        self._count = 0
        return self
    def switch_leg(self):
        self.which_leg *= -1

    def get_gait_pattern(self, landing_position, step, next_step):
        self.switch_leg()
        next_landing_position = np.array([
            landing_position[0] + step[0],
            landing_position[1] - self.which_leg*step[1]
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
    
    def generate_gait_pattern(self, landing_position, step, next_step, dt=1e-3):
        for i in range(int(self.support_phase_time/dt)):
            cog_accelration = self.inverted_pendulum_equation(self.cog_position, landing_position)
            self.cog_velocity += cog_accelration * dt
            self.cog_position += self.cog_velocity * dt

        next_landing_position, next_cog_position, next_cog_velocity  = self.get_gait_pattern(landing_position, step, next_step)

        desired_cog_position = next_landing_position + next_cog_position
        desired_cog_velocity = next_cog_velocity

        # Evaluation gain
        a = 10.0
        b = 1.0
        # Revised landing position
        D = (a*((self.C -1)**2)) + (b*((self.S/self.time_constant)**2))
        landing_position = (-a*((self.C-1)/D))*(desired_cog_position - (self.C*self.cog_position) - (self.time_constant*self.S*self.cog_velocity))\
            - ((b*self.S)/(self.time_constant*D))*(desired_cog_velocity - (self.S/self.time_constant*self.cog_position) - (self.C*self.cog_velocity))
        
        return landing_position , next_landing_position

    def inverted_pendulum_equation(self, cog_position, landing_position):
        # Equation of an inverted pendulum
        return (self.gravity/self.COG_Z)*(cog_position-landing_position) # m/s^2
if __name__ == '__main__':
    
    gp = GaitPattern(9.8)
    
    step_params = [[0,0.1]] +[ [0.1,0.1] ]*100
    landing_position = np.array([0,0.0])
    next_position = np.array([0,0])
    if 0:
        for i in range(len(step_params)-1):
            next_landing_position, next_cog_position, next_cog_velocity = gp.get_gait_pattern(landing_position,step_params[i], step_params[i+1]  )
            landing_position = next_landing_position
            #print(next_cog_velocity)
            print(next_landing_position)

    if 1:
        
        x = []
        land_pos = []
        next_pos = []
        for i in range(len(step_params)-1):
            
            x.append(i)
            land_pos.append(landing_position)
            next_pos.append(next_position)
            landing_position, next_position = gp.generate_gait_pattern(landing_position,step_params[i], step_params[i+1])
        x.append(i+1)
        land_pos.append(landing_position)
        next_pos.append(next_position)


        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(14, 10), facecolor='lightblue')

        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)

        ax1.plot([row[0] for row in land_pos], [row[1] for row in land_pos], 'bo')
        ax1.plot([row[0] for row in next_pos], [row[1] for row in next_pos], 'r+')
        #ax2.plot([row[0] for row in land_pos], [row[1] for row in land_pos], 'bo')
        ax1.grid(color='r', linestyle='dotted', linewidth=1)
        plt.show()
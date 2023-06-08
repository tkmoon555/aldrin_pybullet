import numpy as np



class GaitController:
    def __init__(self, legs):
        self.legs = legs

        

        self.cog_and_zmp_feedback = CogAndZmpFeedback()
        self.abstract_robot = AbstractRobot()
        #self.gait_pattern_generator = 
    def walk(self):
        # Placeholder for walking algorithm using leg movements
        gait_pattern 
        target_zmp = 


class CogAndZmpFeedback:
    def __init__(self):

        # pattern generator
        self.pg_COG = self.pg_COG_velocity = self.pg_ZMP = np.array([0., 0., 0.])

    def set_pattern(self):
        self.pg_COG = np.array([0.01, 0., 0.])
        self.pg_COG_velocity = np.array([0.01, 0., 0.])
        self.pg_ZMP = np.array([0.01, 0., 0.])

    def calculate_disired_zmp(self, position, velocity, zmp):
        k1 = k2 = k3 = 1.
        return self.pg_ZMP - k1*(position-self.pg_COG) - k2*(velocity-self.pg_COG_velocity) - k3*(zmp-self.pg_ZMP)


class AbstractRobot():
    def __init__(self, sampling_time)
        self._zmp_distribution = ZmpDistribution()
        self._ground_reaction_controller = GroundReactionController(sampling_time)

    def get_cog_and_zmp(self):
        
        return None
    def _get_ground_reaction_force(self):
        return None

    def _get_joint_angles(self):
        return None
    
    def _set_target_joint_angles(self,target_joint_angles):
        return None
    
    def distribute_zmp(self, desired_zmp):
        right_zmp = self.get_right_zmp()
        left_zmp 
        alpha
        ground_reaction_force
    
class ZmpDistribution:
    def __init__(self):
        pass

    def distribute(self, right_zmp, left_zmp, desired_zmp, alpha, ground_reaction_force):
        desired_right_ground_reaction_force = alpha * ground_reaction_force
        desired_left_ground_reaction_force = (
            1-alpha) * ground_reaction_force

        desired_momentum = np.cross(desired_zmp, ground_reaction_force) \
            - np.cross(right_zmp, desired_right_ground_reaction_force) \
            - np.cross(left_zmp,
                        desired_left_ground_reaction_force)

        right_momentum = alpha * desired_momentum
        left_momentum = (1-alpha) * desired_momentum
        return right_momentum, left_momentum


class GroundReactionController:
    def __init__(self, sampling_time):
        self.__amount_of_rotation = 0.
        self.__ankle_z_control = 0.
        self.dt = sampling_time
    def damping_control(self, right_ground_reaction_force, left_ground_reaction_force,
                         desired_right_ground_reaction_force, desired_left_ground_reaction_force,
                         momentum, desired_momentum, damping_gain=1.0):

        diff_ground_reaction_force = left_ground_reaction_force - right_ground_reaction_force
        diff_desired_ground_reaction_force = desired_left_ground_reaction_force - \
            desired_right_ground_reaction_force

        self.__ankle_z_control += (((diff_desired_ground_reaction_force -
                            diff_ground_reaction_force)/damping_gain) - self.__amount_of_rotation) * self.dt


        self.__amount_of_rotation += (((desired_momentum - momentum) /
                                      damping_gain) - self.__amount_of_rotation) * self.dt

        ankle_z_control = self.__ankle_z_control
        return ankle_z_control

class StabilityControlSystem:
  def __init__(self,sampling_time):
      self._zmp_distribution = ZmpDistribution()
      self._ground_reaction_controller = GroundReactionController(sampling_time)
      self._cog_and_zmp_feedback = CogAndZmpFeedback()


      # This matrix is position and orientation of the Ground coordinate system from the perspective of the self coordinate system
      #homogeneous transformation matrix 
      self.H_matrix = np.array(4,4)

      self.right_zmp = None
      self.left_zmp = None
      self.ground_reaction_force = None

      self.right_ground_reaction_force = None
      self.left_ground_reaction_force = None
  def get_cog_zmp(self):
    pass

  def generate_target_zmp(self):
    pass
  def generate_target_ground_reaction_force(self):
    pass
  def generate_target_robot_angles(self):
    pass
  
  def demo(self):
    cog, zmp = self.get_cog_zmp()
    target_zmp = self.generate_target_zmp(cog, zmp)
    target_ground_reaction_force  = self.generate_target_ground_reaction_force(target_zmp)
    self.generate_target_robot_angle(target_ground_reaction_force, articular_angles, ground_reaction_force)

  def _distribute_zmp(self, desired_zmp, alpha, type='single_leg'): 
    if type == 'single_leg': #単脚支持
      target_right_ground_reaction_force, target_left_ground_reaction_force = self._zmp_distribution.distribute(self.right_zmp, self.left_zmp, desired_zmp, alpha, self.ground_reaction_force)
    #TODO elif type == 'both_leg': #両脚支持
    else:
      print("distribute zmp's type error")
      exit()  
    return target_right_ground_reaction_force, target_left_ground_reaction_force
  
  def control_ground_reaction_force(self, \
                                    desired_ground_reaction_force, \
                                    joint_angle_pattern, \
                                        ground_reaction_force):
    desired_right_ground_reaction_force = desired_ground_reaction_force[0]
    desired_left_ground_reaction_force = desired_ground_reaction_force[1]

    
    

    self._ground_reaction_controller.damping_control(self.right_ground_reaction_force, self.left_ground_reaction_force,
                         desired_right_ground_reaction_force, desired_left_ground_reaction_force,
                         momentum, desired_momentum, damping_gain=1.0)
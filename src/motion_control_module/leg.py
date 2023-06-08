
"""
class Leg:
    def __init__(self, kinematics, joint_angles=None):
        self.kinematics = kinematics
        self.joint_angles = joint_angles or [0, 0, 0]  # Default joint angles

    def move_to(self, joint_angles):
        self.joint_angles = joint_angles
        # Calculate end effector position and orientation using forward kinematics
        position, orientation = self.kinematics.forward_kinematics(joint_angles)
        # Move leg to calculated position and orientation
        # (implementation omitted)

"""

class Leg:
    def __init__(self, joints, joint_angles):
        self.kinematics = kinematics
        self.joint_angles = joint_angles  # Default joint angles

        self.joints = joints



    def move_to(self, joint_angles):
        self.joint_angles = joint_angles

        for i,angle in enumerate(joint_angles):
          self.joints[i].set_position(angle)
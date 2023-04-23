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

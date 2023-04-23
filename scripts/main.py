"""
このスクリプトでは、まず、ロボットのURDFファイルのパスを定義します。次に、PyBulletSimulatorのインスタンスを作成し、ロボットモデルをロードし、ロボットの初期位置と姿勢を設定します。

次にRobotControllerインスタンスを作成し、1000回のタイムステップでシミュレーションを実行します。各時間ステップで、PyBulletSimulatorインスタンスのget_robot_positionメソッドでロボットの現在の位置と姿勢を取得し、RobotControllerインスタンスのcompute_control_signalsメソッドで制御信号を計算し、PyBulletSimulatorインスタンスのapply_control_signalsメソッドでロボットに制御信号を適用しています。次に、PyBulletSimulatorインスタンスのstep_simulationメソッドを用いて、シミュレーションを1時間進めます。

1000回のシミュレーションを行った後、PyBulletSimulatorインスタンスのget_robot_positionメソッドでロボットの最終位置と姿勢を取得し、最終位置と姿勢を出力し、PyBulletSimulatorインスタンスのcloseメソッドでシミュレータを終了します。

"""


"""
from pybullet_simulator import PyBulletSimulator
from controller import RobotController

# Define the URDF file path for the robot
urdf_file_path = "path/to/robot.urdf"

# Create a PyBulletSimulator instance
simulator = PyBulletSimulator()

# Load the robot model
simulator.load_robot(urdf_file_path)

# Set the initial position and orientation of the robot
initial_position = [0, 0, 1]
initial_orientation = [0, 0, 0, 1]
simulator.set_robot_position(initial_position, initial_orientation)

# Create a RobotController instance
controller = RobotController()

# Run the simulation for 1000 time steps
num_steps = 1000
for i in range(num_steps):
    # Get the current position and orientation of the robot
    current_position, current_orientation = simulator.get_robot_position()
    
    # Compute the control signals
    control_signals = controller.compute_control_signals(current_position, current_orientation)
    
    # Apply the control signals to the robot
    simulator.apply_control_signals(control_signals)
    
    # Step the simulation forward one time step
    simulator.step_simulation()

# Get the final position and orientation of the robot
final_position, final_orientation = simulator.get_robot_position()

# Print the final position and orientation of the robot
print("Final position: ", final_position)
print("Final orientation: ", final_orientation)

# Close the simulator
simulator.close()
"""

import pybullet as p
import pybullet_data
import time
import os 
import sys


#from biped_model import BipedModel

from simulator.pybullet.simulation_pybullet import SimulationPybullet

if __name__ == '__main__':
    pkg_path = os.path.abspath('..')
    sim = SimulationPybullet(pkg_path)

    sim.run()
"""
このスクリプトでは、まず、ロボットのURDFファイルのパスを定義します。次に、PyBulletSimulatorのインスタンスを作成し、ロボットモデルをロードし、ロボットの初期位置と姿勢を設定します。

そして、run_simulationメソッドで1000回のシミュレーションを行い、get_robot_positionメソッドでロボットの最終位置と姿勢を取得します。最終的なロボットの位置と姿勢を出力し、closeメソッドでシミュレータを終了します。
"""



from pybullet_simulator import PyBulletSimulator

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

# Run the simulation for 1000 time steps
num_steps = 1000
simulator.run_simulation(num_steps)

# Get the final position and orientation of the robot
final_position, final_orientation = simulator.get_robot_position()

# Print the final position and orientation of the robot
print("Final position: ", final_position)
print("Final orientation: ", final_orientation)

# Close the simulator
simulator.close()

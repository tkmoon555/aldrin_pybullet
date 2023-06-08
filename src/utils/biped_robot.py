import abc
from .biped_model import BipedModel


class AbstractRobot(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def start_walking(self):
        pass

    @abc.abstractmethod
    def stop_walking(self):
        pass

    @abc.abstractmethod
    def step_forward(self):
        pass

    @abc.abstractmethod
    def step_backward(self):
        pass 
 

class Robot:
    def __init__(self, name, movement_type, sensors=None, motors=None):

        self.movement_type = movement_type
        self.sensors = sensors or []
        self.motors = motors or []

        self.name = name
        #self.urdf = urdfpy.URDF.load(urdf_path)
        self.is_on = False
        self.ZMP = None
        self.COG = None
        #self.kdl_model = BipedKdlModel(urdf_path)

    def turn_on(self):
        self.is_on = True
        print(f"{self.name} is now on.")

    def turn_off(self):
        self.is_on = False
        print(f"{self.name} is now off.")

    def move(self):
        pass  # Placeholder for the movement logic

    def sense(self):
        pass  # Placeholder for the sensing logic

class BipedRobot(Robot, AbstractRobot):
    def __init__(self, name, urdf_path):
        super().__init__(name, urdf_path)
        #self.leg_count = self.urdf.get_num_legs()

        # Extract information about robot's mass and geometry from URDF
        #self.mass = self.urdf.get_mass()
        #self.inertia = self.urdf.get_inertia_tensor()
        #self.link_com = self.urdf.get_link_com()

    def start_walking(self):
        print(f"{self.name} has started walking.")

    def stop_walking(self):
        print(f"{self.name} has stopped walking.")

    def step_forward(self):
        print(f"{self.name} has stepped forward.")

    def step_backward(self):
        print(f"{self.name} has stepped backward.")



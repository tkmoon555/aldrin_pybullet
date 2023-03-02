import numpy as np

VOLTAGE_CLIPPING = 24

class BLDCMotor:
    def __init__(self, name, motor_voltage, resistance, inductance, inertia, damping,
                 kp=1.0, kd=0, kt = 0.5, 
                 max_current=10, max_rpm=1e+5):
        self.__name = name
        #TODO self.__max_rpm = max_rpm
        self.__motor_voltage = motor_voltage
        self._max_current = max_current

        #initialize constant params
        self._w_ref = 0         
        self._kp = kp   #proportional gain
        self._kd = kd   #differential gain
        self._ke = self._kt = kt   #torque constant
        #TODO self._ke = kt + viscous_damping

        self.__angle = 0.0
        self.__angular_velocity = 0.0
        self.__angular_acceleration = 0.0

        self._rl_circuit =  RLCircuit(resistance, inductance)
        self._motor_dynamics = MotorDynamics(inertia, damping)

    def set_speed(self, speed_rpm):
        #TODO code to set the speed of the motor in RPM
        pass

    def request_angle(self, target_angle, load_torque):
        # code to set and get angle, w, surplus_torque of the motor
        assert target_angle <= 2*np.pi, 'target_angle : {} may be a degree.'.format(target_angle)
        error_angle = target_angle - self.angle_radian
        pwm = self._kp * error_angle + \
              self._kd*(self._w_ref - self.angular_velocity)
        voltage = np.clip(self.motor_voltage * pwm, 
                          -VOLTAGE_CLIPPING, 
                          VOLTAGE_CLIPPING)  - (self._ke * self.angular_velocity)
                        
        surplus_torque = (self._kt * self._rl_circuit.get_current(voltage)) +load_torque #(self._kt * current)  is electrical torque
        self.__angular_velocity = self._motor_dynamics.get_angular_velocity(surplus_torque)
        self.__angle += self.angular_velocity
        return self.angle_radian, self.angular_velocity, surplus_torque
    
    def _velocity_control(self, target_velocity_radian_second):
        #TODO 
        pass

    @property
    def name(self):
        return self.__name
    @property
    def motor_voltage(self):
        return self.__motor_voltage
    @motor_voltage.setter
    def motor_voltage(self,voltage):
        print("updte voltage from {} to {}.".format(self.__motor_voltage, voltage))
        self.__motor_voltage = voltage

    @property
    def angular_acceleration(self):
        return self.__angular_acceleration
    @property
    def angle_radian(self):
        return self.__angle
    @property
    def angular_velocity(self):
        # code to get the current motor angle velocity at radian / second
        return self.__angular_velocity
    

class RLCircuit:
    def __init__(self, resistance, inductance):
        self._resistance = resistance
        self._inductance = inductance
        self.__current = 0.0
    
        self.__integral = 0.0
    def get_current(self, voltage):
        # code to get the current through the circuit
        self.__integral += voltage - (self.__current*self._resistance)
        self.__current = self.__integral / self._inductance
        return self.__current

class MotorDynamics:
    def __init__(self, inertia, damping):
        self._inertia = inertia
        self._damping = damping
        self.__angular_velocity = 0.0

        self.__integral = 0.0
    def get_angular_velocity(self, torque):
        # code to get the current angular velocity of the motor
        self.__integral += torque - self._damping * self.__angular_velocity
        self.__angular_velocity =  self.__integral / self._inertia
        return self.__angular_velocity


if __name__ == '__main__':
    bldc = BLDCMotor("motor1", 24,1,1,1,1)
    bldc.request_angle(2*np.pi, -0.1)


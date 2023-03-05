import numpy as np

VOLTAGE_CLIPPING = 24
SAMPLING_TIME = 1e-5  # second


class BLDCMotor:
    def __init__(self, id, motor_voltage, resistance, inductance, inertia, damping,
                  kt, 
                  kp=1.0, kd=0.0,
                 max_current=10.0, max_rpm=1e+5):
        self.__id = id
        # TODO self.__max_rpm = max_rpm
        self.__motor_voltage = motor_voltage
        self._max_current = max_current

        # initialize constant params
        self._w_ref = 0.0
        self._kp = kp  # proportional gain
        self._kd = kd  # differential gain
        self._ke = self._kt = kt  # torque constant
        # TODO self._ke = kt + viscous_damping

        self.__angle = 0.0
        self.__angular_velocity = 0.0
        self.__angular_acceleration = 0.0

        self._rl_circuit = RLCircuit(resistance, inductance)
        self._motor_dynamics = MotorDynamics(inertia, damping)

    def set_speed(self, speed_rpm):
        # TODO code to set the speed of the motor in RPM
        pass

    def request_angle(self, target_angle, load_torque):
        # code to set and get angle, w, surplus_torque of the motor
        assert target_angle <= 2 * np.pi, \
            'target_angle : {} may be a degree.'.format(target_angle)
        error_angle = target_angle - self.angle_radian
        pwm = self._kp * error_angle + \
            self._kd*(self._w_ref - self.angular_velocity)
        voltage = np.clip(self.motor_voltage * pwm,
                          -VOLTAGE_CLIPPING,
                          VOLTAGE_CLIPPING) - (self._ke * self.angular_velocity)

        # (self._kt * current)  is electrical torque
        surplus_torque = (
            self._kt * self._rl_circuit.get_current(voltage)) + load_torque
        self.__angular_velocity = self._motor_dynamics.get_angular_velocity(
            surplus_torque)
        self.__angle += self.angular_velocity * SAMPLING_TIME
        return self.angle_radian, self.angular_velocity, surplus_torque

    def _velocity_control(self, target_velocity_radian_second):
        # TODO
        pass

    @property
    def id(self):
        return self.__id

    @property
    def motor_voltage(self):
        return self.__motor_voltage

    @motor_voltage.setter
    def motor_voltage(self, voltage):
        print("updte voltage from {} to {}.".format(
            self.__motor_voltage, voltage))
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
        self.__integral += (voltage - (self.__current *
                            self._resistance))*SAMPLING_TIME
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
        self.__integral += (torque - self._damping *
                            self.__angular_velocity)*SAMPLING_TIME
        self.__angular_velocity = self.__integral / self._inertia
        return self.__angular_velocity


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    # R16 bldc moon's
    resistance = 2.2        #Ω
    inductance = 0.3*1e-3   #H
    J = 2.86 * 1e-7         #kg m^2
    damping = 10 * J        
    kt = 5.04*1e-3          #Nm / A
    
    #set control params
    kp = 0.5
    kd = 0.001

    bldc = BLDCMotor("motor1", 10, resistance, inductance, J, damping, kt, kp, kd)

    target_angle = 1/2 *np.pi

    x = []
    y1 = []
    y2 = []
    y3 = []
    for i in range(int(0.1 / SAMPLING_TIME)):
        angle, angle_vel, torque = bldc.request_angle(target_angle, 0)

        x.append(i*SAMPLING_TIME)

        y1.append(angle*360/(2*np.pi))
        y2.append(torque)
        y3.append(angle_vel*60/(2*np.pi))


    print("Electrical time constant : {}(ms), Mechanical time constant : {}(ms)".format(inductance/resistance, J/damping))
    print("target angle {} (degree)".format(target_angle * 360/(2*np.pi)))

    fig = plt.figure(figsize=(14, 10), facecolor='lightblue')

    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2)
    ax3 = fig.add_subplot(3, 1, 3)

    ax1.plot(x, y1,  label="angle (degree)")
    ax2.plot(x, y2, label="torque (Nm)")
    ax3.plot(x, y3,  label="angle velocity (r/min)")

    ax1.set_ylabel("angle (degree)")
    ax2.set_ylabel("torque (Nm)")
    ax3.set_ylabel("angle velocity (r/min)")

    ax1.set_xlabel("time (second)")
    ax2.set_xlabel("time (second)")
    ax3.set_xlabel("time (second)")

    plt.show()

import cv2

class BipedSensors:
    def __init__(self):
        # initialize camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # initialize other sensors here
        '''TODO
        self.gyroscope = Gyroscope()
        self.accelerometer = Accelerometer()
        '''

    def get_camera_frame(self):
        # read a frame from the camera and return it
        ret, frame = self.camera.read()
        if ret:
            return frame
        else:
            return None

    def get_gyroscope_reading(self):
        # read data from gyroscope and return it
        return self.gyroscope.read()

    def get_accelerometer_reading(self):
        # read data from accelerometer and return it
        return self.accelerometer.read()

    def read_angular_velocity(self):
        # Read and return the angular velocity data from the gyroscope
        return self.gyroscope.read()

    def read_force(self):
        # Read and return the force data from the force sensor
        return self.force_sensor.read()
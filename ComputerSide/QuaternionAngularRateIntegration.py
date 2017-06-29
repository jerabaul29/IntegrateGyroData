from __future__ import print_function
import ReadStructFromArduino as rsfa
import serial
import Quaternions as qt
import PygameRendering as PR


class GYR_Integration(object):
    def __init__(self, ReadFromArduino_instance, dt, offset_GYRO, verbose=0):

        self.dt = dt
        self.verbose = verbose
        self.ReadFromArduino_instance = ReadFromArduino_instance
        self.list_measurements = []
        self.offset_GYRO = offset_GYRO

        # initial state: assume resting, Z downwards, still --------------------
        # no rotation to Earth referential
        self.o = qt.normalise_quaternion(qt.Quaternion(1, 0.0001, 0.0001, 0.0001))

        # reference vectors; can be used to print assumed orientation ----------
        self.X_IMU_ref_IMU = qt.Vector(1, 0, 0)
        self.Y_IMU_ref_IMU = qt.Vector(0, 1, 0)
        self.Z_IMU_ref_IMU = qt.Vector(0, 0, 1)

        self.read_and_update_measurement()

        self.cycle = 0

    def predict_state(self):
        """predict quaternion orientation. Use a First order finite difference
        for integration of sensor motion. The integration input is the
        gyroscope signal"""

        qDelta = qt.angular_rate_to_quaternion_rotation(self.wm, self.dt)
        self.o = qt.quaternion_product(self.o, qDelta)

        if self.verbose > 2:
            print("PRINT NEW STATE ###")
            self.print_state(self.o)

    def read_and_update_measurement(self):
        """Read and update measurement from Arduino."""

        self.ReadFromArduino_instance.read_one_value()
        self.latest_measurement = self.ReadFromArduino_instance.latest_values
        self.wm = qt.Vector(self.latest_measurement[3] - self.offset_GYRO[0],
                            self.latest_measurement[4] - self.offset_GYRO[1],
                            self.latest_measurement[5] - self.offset_GYRO[2])

        if self.verbose > 2:
            print("Obtained measurements in SI:")
            print(self.latest_measurement)

        self.list_measurements.append(self.latest_measurement)

    def perform_one_iteration(self):
        """Perform one integration iteration: read, integrate to compute update,
        and print if necessary."""

        if self.verbose > 0:
            print("\n### NEW CYCLE " + str(self.cycle) + " ###\n")

        self.predict_state()
        self.read_and_update_measurement()

        # normalise orientation quaternion: this can be necessary if some noise
        # due to for exemple nearly singular matrix that get inverted
        self.o = qt.normalise_quaternion(self.o)

        if self.verbose > 0:
            print("PRINT STATE AT END INTEGRATION CYCLE ###")
            self.print_state(self.o)

        self.cycle += 1

    def print_state(self, o):
        """Print information about state of the quaternion"""

        current_X_IMU = qt.apply_rotation_on_vector(o, self.X_IMU_ref_IMU)
        current_Y_IMU = qt.apply_rotation_on_vector(o, self.Y_IMU_ref_IMU)
        current_Z_IMU = qt.apply_rotation_on_vector(o, self.Z_IMU_ref_IMU)

        print("Print state of quaternion --------------------------------------------")

        print("state orientation: current X, Y, Z of the IMU, in referential Earth:")
        qt.print_vector(current_X_IMU)
        qt.print_vector(current_Y_IMU)
        qt.print_vector(current_Z_IMU)

        return(current_X_IMU, current_Y_IMU, current_Z_IMU)

common_dt = 0.02

ports = rsfa.look_for_available_ports()

usb_port = serial.Serial(ports[0], baudrate=115200, timeout=0.5)

read_from_Arduino_instance = rsfa.ReadFromArduino(usb_port, verbose=0)

# determined by calibration: offset of the gyro; take away to reduce drift.
offset_GYRO = [-0.029, -0.008, 0.013]
GYR_Integration_instance = GYR_Integration(read_from_Arduino_instance, dt=common_dt, offset_GYRO=offset_GYRO, verbose=3)

RenderGyroIntegration_instance = PR.RenderGyroIntegration(GYR_Integration_instance)
RenderGyroIntegration_instance.run()

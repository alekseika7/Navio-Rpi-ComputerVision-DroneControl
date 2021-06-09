#!python3

from time import sleep

from dronekit import connect, VehicleMode
from pymavlink import mavutil


class Vehicle():

    def __init__(self, address='127.0.0.1:14550'):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0
        self.yaw_speed = 50
        self._enable_flag = False
        self.mov_speed = 0.2
        self.takeoff_height = 2
        self._copter = connect(address, wait_ready=True)

    def _perform_mov(self):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        if self._enable_flag:
            if self.yaw == 0:
                msg = self._copter.message_factory.set_position_target_local_ned_encode(
                    0,  # time_boot_ms
                    0, 0,   # target system, target component
                    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
                    0b0000111111000111,  # type_mask
                    0, 0, 0,  # x, y, z positions
                    self.vx, self.vy, self.vz,  # x, y, z velocity in m/s
                    0, 0, 0,  # x, y, z acceleration
                    0, 0)  # yaw, yaw_rate
            else:
                msg = self._copter.message_factory.command_long_encode(
                        0, 0,  # target system, target component
                        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                        0,  # confirmation
                        50,    # yaw in degrees
                        self.yaw_speed,  # yaw speed deg/s
                        self.yaw,  # direction -1 ccw, 1 cw
                        1,  # relative offset 1, absolute angle 0
                        0, 0, 0)  # param 5 ~ 7 not used

            for i in range(7):
                self._copter.send_mavlink(msg)

        self._enable_flag = False

    def forward(self):
        """
        Move drone forward (vx > 0)
        """
        self.vx = self.mov_speed
        self._perform_mov()
        print("Moving front...")

    def backward(self):
        """
        Move drone backward (vx < 0)
        """
        self.vx = -self.mov_speed
        self._perform_mov()
        print("Moving front...")

    def up(self):
        """
        Move drone up (vz < 0)
        """
        self.vz = -self.mov_speed
        self._perform_mov()
        print("Moving up...")

    def down(self):
        """
        Move drone down (vz > 0)
        """
        self.vz = self.mov_speed
        self._perform_mov()
        print("Moving down...")

    def left(self):
        """
        Move drone left (vy < 0)
        """
        self.vy = -self.mov_speed
        self._perform_mov()
        print("Moving left...")

    def right(self):
        """
        Move drone right (vy > 0)
        """
        self.vy = self.mov_speed
        self._perform_mov()
        print("Moving right...")

    def clockwise(self):
        """
        Move drone clockwise (yaw = 1)
        """
        self.yaw = 1
        self._perform_mov()
        print("Rotating clockwise...")

    def counterclockwise(self):
        """
        Move drone counter-clockwise (yaw = -1)
        """
        self.yaw = -1
        self._perform_mov()
        print("Rotating counter-clockwise...")

    def start_command(self):
        """
        Enable command execution or activate the hand tracking mode
        """
        print("Enable command execution")
        # needs to be modified later in order to perform tracking task
        self._enable_flag = True

    def end_command(self):
        """
        Stop command execution
        """
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.yaw = 0
        self._perform_mov()
        self._enable_flag = False
        print("Stoped")

    def arm(self):
        """
        Arm the copter
        """
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self._copter.is_armable:
            print("Waiting for vehicle to initialize...")
            sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self._copter.mode = VehicleMode("GUIDED")
        self._copter.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self._copter.armed:
            print("Waiting for arming...")
            sleep(1)

        print("Armed")

    def disarm(self):
        print("Disarming motors")
        self._copter.armed = False

        while self._copter.armed:
            print("Disarming...")
            sleep(1)

        print("Disarmed")

    def landing(self):
        print("Landing...")
        self._copter.mode = VehicleMode("LAND")
        while True:
            if self._copter.location.global_relative_frame.alt <= 0.1:
                break
                sleep(1)

        self._enable_flag = False
        print("Ground is reached")

    def takeoff(self):
        print("Taking off...")

        if not self._copter.armed:
            self.arm()

        self._copter.simple_takeoff(self.takeoff_height)
        while True:
            if self._copter.location.global_relative_frame.alt >= self.takeoff_height * 0.95:
                break
            sleep(1)

        self._enable_flag = False
        print("Target height is reached")


if __name__ == "__main__":
    try:
        vehicle = Vehicle()
        vehicle.connect_drone()
        vehicle.arm()
        vehicle.takeoff()
        vehicle.landing()
        vehicle.disarm()
    except Exception as e:
        print(e)


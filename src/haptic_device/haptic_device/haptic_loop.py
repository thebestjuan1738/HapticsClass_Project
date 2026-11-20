
import time
import math

import pyOpenHaptics.hd as hd
from pyOpenHaptics.hd_callback import hd_callback, hdAsyncSheduler

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point
from ctypes import c_int


# Each update replaces this with a complete (timestamp, x, y, z) tuple.
latest_sample = None
callback_error = None

start_time = time.perf_counter()
prev_time = start_time

class Haptic_Node(Node):
    def __init__(self):
        super().__init__('haptic_node')
        # Initialize Node publishers
        self.publisher_haptics = self.create_publisher(Float32MultiArray, 'haptic_topic', qos_profile_sensor_data)

        self.subscriber_interface = self.create_subscription(String, 'interface_topic', self.interface_callback, qos_profile_sensor_data)
        
        # Set haptic environment variables
        self.k = 0.5
        self.b = 0.0015
        self.start_wall_flag = 0
        self.page_wall = -80

        self.curr_data = None

        self.curr_screen = None

        @hd_callback
        def haptic_update():
            self.curr_data = self.update_haptics()

        self.haptic_callback = haptic_update

        self.publish_timer = self.create_timer(0.05, self.publish_data)

        # Ensuring that communication is still present
        self.device_id = None
        self.scheduler_started = False
        self.reconnecting = False

        self.connection_timer = self.create_timer(0.05, self.check_connection)

        # Log timer of the haptic device
        # self.time_timer = self.create_timer(0.1, self.log_time)

    def log_time(self):
        sample = self.curr_data
        self.get_logger().info('Current Time: %f' % sample[6])


    def update_haptics(self):

        if self.curr_screen == "rest":
            force_a = [0, 0, 0]
            return([float(position[0]),float(position[1]),float(position[2]),
                    float(force_a[0]),float(force_a[1]),float(force_a[2]),
                    float(curr_time),
                    button_pressed])
        
        position = hd._get_doublev(hd.HD_CURRENT_POSITION, hd.hduVector3Dd)

        velocity = hd._get_doublev(hd.HD_CURRENT_VELOCITY, hd.hduVector3Dd)

        rotation = hd._get_doublev(hd.HD_CURRENT_GIMBAL_ANGLES, hd.hduVector3Dd)


        button_press = hd._get_integerv(hd.HD_CURRENT_BUTTONS, c_int)
        button_pressed = bool(button_press.value & 1)
        
        force_a = [0, 0, 0]
        if self.start_wall_flag == 1:
            if position[1] < self.page_wall:
                # force_a = [0.0, k*(position[1]-wall_distance)*wall_normal[1], k*(position[2]-wall_distance)*wall_normal[2]]
                force_a = [-self.b*velocity[0], 
                               min(3.3,self.k*(self.page_wall-position[1])), 
                               -self.b*velocity[2] ]
        elif position[1] > self.page_wall:
            self.start_wall_flag = 1

        #self.get_logger().info()
    
        hd.set_force(force_a)

        curr_time = time.perf_counter() - start_time

        return([float(position[0]),float(position[1]),float(position[2]),
                float(rotation[0]),float(rotation[1]),float(rotation[2]),
                float(force_a[0]),float(force_a[1]),float(force_a[2]),
                float(curr_time),
                button_pressed])

    def publish_data(self):
        # Will not publish if data is empty or system is reconnecting
        if self.reconnecting or hd.last_error is not None:
            return

        sample = self.curr_data
        if sample is None:
            return  # The haptic callback has not produced data yet.

        msg_haptics = Float32MultiArray()
        msg_haptics.data =  (sample[0], sample[1], sample[2],   # Positional data
                            sample[3], sample[4], sample[5],    # Rotational data (x-z plane, y-z plane, x-y plane)
                            sample[6], sample[7], sample[8],    # Applied linear force data
                            sample[9],                          # Timestamp
                            1.0 if sample[10] else 0.0)         # Button pressed boolean
        self.publisher_haptics.publish(msg_haptics)

    def interface_callback(self,msg):
        self.curr_screen = msg.data[0]

    def open_device(self):
        # Only call this when the previous session has been closed.
        hd.last_error = None
        self.curr_data = None
        self.start_wall_flag = 0

        self.device_id = hd.init_device("Default Device")

        # Check the driver error even if init_device returned a handle.
        initialization_error = hd.get_error()

        if (
            self.device_id is None
            or self.device_id == hd.HD_INVALID_HANDLE
            or initialization_error
        ):
            raise RuntimeError(
                f"Device initialization failed: {hd.last_error}"
            )

        hd.enable_force()

        if hd.get_error():
            raise RuntimeError(
                f"Could not enable forces: {hd.last_error}"
            )

        self.haptic_scheduler = hdAsyncSheduler(
            self.haptic_callback
        )

        hd.start_scheduler()
        self.scheduler_started = True

        if hd.get_error():
            raise RuntimeError(
                f"Scheduler startup failed: {hd.last_error}"
            )


    def close_device(self):
        if self.scheduler_started:
            hd.stop_scheduler()
            self.scheduler_started = False

        # The scheduler is stopped before touching shared device state.
        self.curr_data = None
        self.start_wall_flag = 0

        if (
            self.device_id is not None
            and self.device_id != hd.HD_INVALID_HANDLE
        ):
            hd.close_device(self.device_id)

        self.device_id = None


    def check_connection(self):
        error = hd.last_error

        if self.reconnecting or error is None:
            return

        if error[0] != 770:
            return

        self.reconnecting = True

        self.get_logger().warning(
            f"Communication failed: {error}. Reconnecting..."
        )

        try:
            # Waits for the old haptic execution to stop before resetting it.
            self.close_device()
            self.open_device()

            self.get_logger().info(
                "Device reopened; waiting for fresh measurements."
            )

        finally:
            self.reconnecting = False


def main(args=None):
    rclpy.init(args=args)

    try:
        # device_id = hd.init_device("Default Device")

        # if device_id is None:
        #     raise RuntimeError("Could not open Default Device.")

        # if hd.get_error():
        #     raise RuntimeError("OpenHaptics initialization error.")

        haptic_node = Haptic_Node()
        haptic_node.open_device()

        # hd.enable_force()

        # # Retain the scheduler registration while the node runs.
        # haptic_node.haptic_scheduler = hdAsyncSheduler(
        #     haptic_node.haptic_callback
        # )

        # hd.start_scheduler()
        # scheduler_started = True

        rclpy.spin(haptic_node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop device callbacks before closing the device.
        # try:
        #     if scheduler_started:
        #         hd.stop_scheduler()
        # finally:
        try:
            if haptic_node is not None:
                haptic_node.close_device()
        finally:
            if haptic_node is not None:
                haptic_node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()



if __name__ == "__main__":
    main()
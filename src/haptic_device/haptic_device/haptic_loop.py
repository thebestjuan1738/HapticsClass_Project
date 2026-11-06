
import time
import math

import pyOpenHaptics.hd as hd
from pyOpenHaptics.hd_callback import hd_callback, hdAsyncSheduler

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool
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
        self.publisher_pos = self.create_publisher(Point,'pos_topic',10)    # Publishes position
        self.publisher_force = self.create_publisher(Point,'force_topic',10)    # Publishes forces
        self.publisher_time = self.create_publisher(Float32,'time_topic',10)    # Publishes time of simulation
        
        self.publisher_button = self.create_publisher(Bool, 'button_topic',10)  # Publishes buttons pressed on device
        
        # Set haptic environment variables
        self.k = 0.5
        self.start_wall_flag = 0
        self.page_wall = -40

        self.curr_data = None

        @hd_callback
        def haptic_update():
            self.curr_data = self.update_haptics()

        self.haptic_callback = haptic_update

        self.publish_timer = self.create_timer(0.01, self.publish_data)


    def update_haptics(self):
    
        position = hd._get_doublev(hd.HD_CURRENT_POSITION, hd.hduVector3Dd)

        button_press = hd._get_integerv(hd.HD_CURRENT_BUTTONS, c_int)
        button_pressed = bool(button_press.value & 1)
        
        force_a = [0, 0, 0]
        if self.start_wall_flag == 1:
            if position[1] < self.page_wall:
                # force_a = [0.0, k*(position[1]-wall_distance)*wall_normal[1], k*(position[2]-wall_distance)*wall_normal[2]]
                force_a = [0.0, self.k*(self.page_wall-position[1]), 0.0]
        elif position[1] > self.page_wall:
            self.start_wall_flag = 1

        #self.get_logger().info()
    
        hd.set_force(force_a)

        curr_time = time.perf_counter() - start_time

        return([float(position[0]),float(position[1]),float(position[2]),
                float(force_a[0]),float(force_a[1]),float(force_a[2]),
                float(curr_time),
                button_pressed])

    def publish_data(self):
        sample = self.curr_data
        if sample is None:
            return  # The haptic callback has not produced data yet.

        msg_pos = Point(x=sample[0], y=sample[1], z=sample[2])
        msg_force = Point(x=sample[3], y=sample[4], z=sample[5])
        msg_time = Float32(data=sample[6])
        
        msg_button = Bool(data=sample[7])

        self.publisher_pos.publish(msg_pos)
        self.publisher_force.publish(msg_force)
        self.publisher_time.publish(msg_time)

        self.publisher_button.publish(msg_button)

        # self.get_logger().info('x position: %f' % msg_pos.x)
        # self.get_logger().info('z position: %f' % msg_pos.z)
        # self.get_logger().info('Button pressed: %s' % msg_button)



def main(args=None):
    rclpy.init(args=args)

    haptic_node = None
    device_id = None
    scheduler_started = False

    try:
        device_id = hd.init_device("Default Device")

        if device_id is None:
            raise RuntimeError("Could not open Default Device.")

        if hd.get_error():
            raise RuntimeError("OpenHaptics initialization error.")

        haptic_node = Haptic_Node()

        hd.enable_force()

        # Retain the scheduler registration while the node runs.
        haptic_node.haptic_scheduler = hdAsyncSheduler(
            haptic_node.haptic_callback
        )

        hd.start_scheduler()
        scheduler_started = True

        rclpy.spin(haptic_node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop device callbacks before closing the device.
        try:
            if scheduler_started:
                hd.stop_scheduler()
        finally:
            try:
                if device_id is not None:
                    hd.close_device(device_id)
            finally:
                if haptic_node is not None:
                    haptic_node.destroy_node()

                if rclpy.ok():
                    rclpy.shutdown()



if __name__ == "__main__":
    main()
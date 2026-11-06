import queue

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

from sensor_bridge.serial_reader import SerialReader


class Sensor_Node(Node):
    """Publishes each field of the Arduino serial stream as sensor/<field>.

    Publishers are created on demand from whatever fields the Arduino
    sends, so adding a sensor only requires changing the Arduino sketch.
    """

    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('port', '')          # '' = auto-detect
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic_prefix', 'sensor')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.prefix = self.get_parameter('topic_prefix').value

        self.publishers_by_field = {}

        self.reader = SerialReader(
            port=port, baud=baud, log=self.get_logger().info)
        self.reader.start()

        # Drain the reader's queue on the ROS thread (rclpy publishers are
        # not guaranteed thread-safe from arbitrary threads).
        self.publish_timer = self.create_timer(0.005, self.publish_data)

    def publish_data(self):
        while True:
            try:
                sample = self.reader.samples.get_nowait()
            except queue.Empty:
                return
            for field, value in sample.items():
                pub = self.publishers_by_field.get(field)
                if pub is None:
                    pub = self.create_publisher(
                        Float32, '%s/%s' % (self.prefix, field), 10)
                    self.publishers_by_field[field] = pub
                    self.get_logger().info(
                        'Publishing %s/%s' % (self.prefix, field))
                pub.publish(Float32(data=float(value)))

    def shutdown(self):
        self.reader.stop()


def main(args=None):
    rclpy.init(args=args)
    node = Sensor_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32, Float64

from sensor_bridge.serial_reader import SerialReader, reduce_samples

SAMPLE_MODES = ('latest', 'mean', 'all')
STATUS_PERIOD_S = 5.0


class Sensor_Node(Node):
    """Reads the Arduino's raw sensor stream over USB serial and publishes it.

    One topic per sensor, named like the haptic node's topics:
        eda_topic, pulse_topic, pressure_topic, temp_topic   (std_msgs/Float32,
        raw ADC counts)
        sensor_time_topic                                    (std_msgs/Float64,
        the Arduino's clock in seconds)
    Any other field the Arduino sends is published as <field>_topic.

    Parameters:
        port             serial port: COM4 (Windows), /dev/ttyACM0 (Linux/WSL).
                         Empty = auto-detect the Arduino.
        baud             serial speed (ignored by the R4's native USB)
        publish_rate_hz  how often to publish. Default 100 Hz, the same rate
                         haptic_loop publishes pos/force/time (its 0.01 s timer),
                         so the streams line up row for row. 0 or negative =
                         publish every Arduino sample (500 Hz).
        sample_mode      what to publish at each tick when the Arduino is faster
                         than publish_rate_hz:
                           'latest'  the newest raw sample (nothing computed)
                           'mean'    the average of the samples since last tick
                           'all'     every sample (forces publishing every sample)
    """

    def __init__(self):
        super().__init__('sensor_node')

        any_type = ParameterDescriptor(dynamic_typing=True)   # accept 100 or 100.0
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200, any_type)
        self.declare_parameter('publish_rate_hz', 100.0, any_type)
        self.declare_parameter('sample_mode', 'latest')

        port = str(self.get_parameter('port').value)
        baud = int(self.get_parameter('baud').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        mode = str(self.get_parameter('sample_mode').value)

        if mode not in SAMPLE_MODES:
            self.get_logger().warn(
                "sample_mode '%s' is not one of %s, using 'latest'" % (mode, SAMPLE_MODES))
            mode = 'latest'
        if rate_hz <= 0:
            mode = 'all'
        if mode == 'all':
            period_s = 0.002      # drain the reader quickly; every sample is published
        else:
            period_s = 1.0 / rate_hz
        self.mode = mode

        self.publisher_time = self.create_publisher(Float64, 'sensor_time_topic', 10)
        self.publishers_by_field = {}
        self.last_overruns = 0

        self.reader = SerialReader(
            port=port, baud=baud, log=self.get_logger().info)
        self.reader.start()

        self.get_logger().info(
            "Reading %s, publishing %s at %s" % (
                port or 'auto-detected port', 'every sample' if mode == 'all' else mode,
                'the Arduino rate' if mode == 'all' else '%.1f Hz' % rate_hz))

        self.published = 0
        self.status_lines = 0
        self.status_published = 0
        self.status_time = time.monotonic()
        self.publish_timer = self.create_timer(period_s, self.publish_data)
        self.status_timer = self.create_timer(STATUS_PERIOD_S, self.report_status)

    def publish_data(self):
        samples = self.reader.samples
        items = []
        while samples:
            try:
                items.append(samples.popleft())
            except IndexError:
                break
        if not items:
            return                # nothing new: stay silent so staleness is visible

        if self.mode == 'all':
            for sample in items:
                self.publish_sample(sample)
        else:
            self.publish_sample(reduce_samples(items, self.mode))

    def publish_sample(self, sample):
        for field, value in sample.items():
            if field == 'ms':
                self.publisher_time.publish(Float64(data=value / 1000.0))
                self.published += 1
            elif field == 'ovr':
                if value > self.last_overruns:
                    self.get_logger().warn(
                        'Arduino schedule slipped (overruns now %d)' % value)
                self.last_overruns = value
            else:
                pub = self.publishers_by_field.get(field)
                if pub is None:
                    topic = '%s_topic' % field
                    pub = self.create_publisher(Float32, topic, 10)
                    self.publishers_by_field[field] = pub
                    self.get_logger().info('Publishing %s' % topic)
                pub.publish(Float32(data=float(value)))
                self.published += 1

    def report_status(self):
        now = time.monotonic()
        dt = now - self.status_time
        lines = self.reader.lines_received
        if dt <= 0:
            return
        if not self.reader.connected or lines == self.status_lines:
            self.get_logger().warn(
                'No data from the Arduino in the last %.0f s (port %s)' % (
                    dt, self.reader.port_in_use or 'not open'))
        else:
            self.get_logger().info(
                'Arduino %.0f lines/s | published %.0f msgs/s | '
                'overflow drops: %d | incomplete lines skipped: %d' % (
                    (lines - self.status_lines) / dt,
                    (self.published - self.status_published) / dt,
                    self.reader.dropped, self.reader.rejected))
        self.status_lines = lines
        self.status_published = self.published
        self.status_time = now

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

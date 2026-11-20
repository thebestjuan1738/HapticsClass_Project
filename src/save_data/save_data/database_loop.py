
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32MultiArray, String, Float32

import sqlite3

SENSORS = ('eda', 'pulse', 'temp')      # raw ADC counts from sensor_bridge (A0, A1, A2)
STATUS_PERIOD_S = 5.0                   # how often missing/stale inputs are reported
STALE_AFTER_S = 2.0                     # a sensor with no new value this long is called stale


class Database_Node(Node):
    def __init__(self):
        super().__init__('database_node')

        # Initialize node subscribers
        self.subscriber_haptics = self.create_subscription(Float32MultiArray,'haptic_topic',self.callback_haptics, qos_profile_sensor_data)
        self.subscriber_interface = self.create_subscription(String, 'interface_topic',self.callback_interface, qos_profile_sensor_data)

        # Sensor topics from sensor_bridge (raw ADC counts). These do NOT decide when a row is
        # written: each row just takes the latest value received (sample-and-hold). Default QoS
        # (reliable) matches the bridge's publishers.
        self.sensor_values = {name: None for name in SENSORS}
        self.sensor_last_seen = {name: None for name in SENSORS}
        self.subscriber_eda = self.create_subscription(
            Float32, 'eda_topic', lambda msg: self.callback_sensor('eda', msg), 1)
        self.subscriber_pulse = self.create_subscription(
            Float32, 'pulse_topic', lambda msg: self.callback_sensor('pulse', msg), 1)
        self.subscriber_temp = self.create_subscription(
            Float32, 'temp_topic', lambda msg: self.callback_sensor('temp', msg), 1)

        # If True (default), no row is written until EVERY sensor above has reported at least
        # once, so a sensor that is unplugged or left out cannot silently produce empty columns.
        # The missing sensor is named in the log every few seconds. Set False to record the
        # haptic data anyway and leave missing sensor columns empty (NULL).
        self.declare_parameter('require_sensors', True)
        self.require_sensors = bool(self.get_parameter('require_sensors').value)

        # Connect to a new database
        self.declare_parameter('database_path', str(Path.cwd() / 'saved_databases' / 'test.db'))
        database_path = Path(self.get_parameter('database_path').value).expanduser()
        database_path.parent.mkdir(parents=True, exist_ok=True)

        database_path.unlink(missing_ok=True)   # Deletes old database if file already present

        self.conn = sqlite3.connect(str(database_path))

        # Build new database structure
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS data(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            screen TEXT NOT NULL,
            time REAL NOT NULL,
            button BOOL NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            z REAL NOT NULL,
            yaw REAL NOT NULL,
            pitch REAL NOT NULL,
            roll REAL NOT NULL,
            F_x REAL NOT NULL,
            F_y REAL NOT NULL,
            F_z REAL NOT NULL,
            eda REAL,
            pulse REAL,
            temp REAL
            )
        """)
        self.conn.commit()

        # Saving a published chunk of data periodically
        self.pending_samples = []   # Initializing vector of tupled samples
        self.curr_screen = None     # Latest screen name from the interface ('design', 'rest', ...)
        self.rows_buffered = 0
        self.save_timer = self.create_timer(0.05, self.save_samples)
        self.status_timer = self.create_timer(STATUS_PERIOD_S, self.report_status)

    def callback_haptics(self,msg):
        self.position = msg.data[0:3]
        self.rotation = msg.data[3:6]
        self.force = msg.data[6:9]
        self.time = msg.data[9]
        self.button = bool(msg.data[10])

        # Each haptic message (20 Hz) is the time base: one message makes one row
        self.buffer_sample()

    def callback_interface(self,msg):
        self.curr_screen = msg.data     # the whole string; msg.data[0] would be one character

    def callback_sensor(self, name, sensor_msg):
        # Only remembers the latest value; the next row written will pick it up
        self.sensor_values[name] = sensor_msg.data
        self.sensor_last_seen[name] = time.monotonic()

    def missing_sensors(self):
        # `is None`, not falsiness: a reading of exactly 0.0 is real data
        return [name for name in SENSORS if self.sensor_values[name] is None]

    def buffer_sample(self):
        # Checks that data has been published for each table column
        if self.curr_screen is None:
            return
        if self.require_sensors and self.missing_sensors():
            return

        # Adds a new tuple of data to pending_samples (same order as the INSERT in save_samples)
        self.pending_samples.append((
            self.curr_screen,
            self.time,
            self.button,
            *self.position,
            *self.rotation,
            *self.force,
            self.sensor_values['eda'],      # latest sensor values (None -> NULL if not received)
            self.sensor_values['pulse'],
            self.sensor_values['temp'],
        ))
        self.rows_buffered += 1

    def report_status(self):
        # Say out loud why rows are not being written, instead of failing silently
        problems = []
        missing = self.missing_sensors()
        if missing:
            problems.append("no data yet from sensor(s): %s (is sensor_bridge running and "
                            "is the Arduino connected?)%s" % (
                                ', '.join(missing),
                                ' - NO ROWS ARE BEING SAVED' if self.require_sensors else ''))
        now = time.monotonic()
        stale = [n for n in SENSORS
                 if self.sensor_last_seen[n] is not None
                 and now - self.sensor_last_seen[n] > STALE_AFTER_S]
        if stale:
            problems.append("sensor(s) stopped updating (rows hold the last value): %s"
                            % ', '.join(stale))
        if self.curr_screen is None:
            problems.append("no message yet on interface_topic (is interface_node running?) - "
                            "NO ROWS ARE BEING SAVED")
        if not hasattr(self, 'position'):
            problems.append("no message yet on haptic_topic (is haptic_loop running?)")
        if problems:
            self.get_logger().warn('; '.join(problems))
        else:
            self.get_logger().info('Rows buffered so far: %d' % self.rows_buffered)

    def save_samples(self):
        # Nothing saved if no data is present
        if not self.pending_samples:
            return

        # Commits buffered samples to the database
        with self.conn:
            self.conn.executemany(      # Reads buffered data into db as row tuples
                """
                INSERT INTO data
                    (screen, time, button, x, y, z, yaw, pitch, roll, F_x, F_y, F_z,
                     eda, pulse, temp)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                self.pending_samples)

        # Clears currently buffered samples
        self.pending_samples.clear()

    def close_database(self):
        try:        # Save the currently queued samples
            self.save_samples()
        finally:    # Close the database connection
            self.conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = Database_Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            # NOTE: this was `node.close_database` without (), which only referenced the method
            # and never ran it, so the samples still buffered at Ctrl+C were never written and the
            # database was never closed cleanly.
            node.close_database()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == "__main__":
    main()

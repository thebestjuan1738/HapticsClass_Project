
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, Int8
from geometry_msgs.msg import Point

import sqlite3



class Database_Node(Node):
    def __init__(self):
        super().__init__('database_node')

        # Initialize node subscribers
        self.subscriber_pos = self.create_subscription(Point,'pos_topic',self.callback_pos,1)
        self.subscriber_force = self.create_subscription(Point,'force_topic',self.callback_force,1)
        self.subscriber_time = self.create_subscription(Float32, 'time_topic',self.callback_time,1)
        self.subscriber_screen = self.create_subscription(Int8, 'screen_topic',self.callback_screen,1)

        # Sensor topics from sensor_bridge (raw ADC counts). These do NOT decide when a row is
        # written: each row just takes the latest value received (sample-and-hold). If the bridge
        # isn't running, those columns are left empty (NULL) and everything else works as before.
        self.sensor_values = {'eda': None, 'pulse': None, 'pressure': None, 'temp': None}
        self.sensor_time = None     # Arduino clock [s] of the latest sensor sample
        self.subscriber_eda = self.create_subscription(
            Float32, 'eda_topic', lambda msg: self.callback_sensor('eda', msg), 1)
        self.subscriber_pulse = self.create_subscription(
            Float32, 'pulse_topic', lambda msg: self.callback_sensor('pulse', msg), 1)
        self.subscriber_pressure = self.create_subscription(
            Float32, 'pressure_topic', lambda msg: self.callback_sensor('pressure', msg), 1)
        self.subscriber_temp = self.create_subscription(
            Float32, 'temp_topic', lambda msg: self.callback_sensor('temp', msg), 1)
        self.subscriber_sensor_time = self.create_subscription(
            Float64, 'sensor_time_topic', self.callback_sensor_time, 1)

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
            screen INTEGER NOT NULL,
            time REAL NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            z REAL NOT NULL,
            F_x REAL NOT NULL,
            F_y REAL NOT NULL,
            F_z REAL NOT NULL,
            eda REAL,
            pulse REAL,
            pressure REAL,
            temp REAL,
            sensor_time REAL
            )
        """)
        self.conn.commit()

        # Saving a published chunk of data periodically
        self.pending_samples = []   # Initializing vector of tupled samples
        self.screen_bool = False    # Initializing boolean checks for data being published
        self.time_bool = False
        self.pos_bool = False
        self.force_bool = False
        self.save_timer = self.create_timer(0.05, self.save_samples)

    def callback_pos(self, pos_msg):
        self.pos = [pos_msg.x,pos_msg.y,pos_msg.z]
        self.pos_bool = True
        self.buffer_sample()

    def callback_force(self, force_msg):
        self.force = [force_msg.x,force_msg.y,force_msg.z]
        self.force_bool = True
        self.buffer_sample()

    def callback_time(self, time_msg):
        self.curr_time = time_msg.data
        self.time_bool = True
        self.buffer_sample()

    def callback_screen(self, screen_msg):
        self.screen = screen_msg.data
        self.screen_bool = True
        self.buffer_sample()

    def callback_sensor(self, name, sensor_msg):
        # Only remembers the latest value; the next row written will pick it up
        self.sensor_values[name] = sensor_msg.data

    def callback_sensor_time(self, time_msg):
        self.sensor_time = time_msg.data

    def buffer_sample(self):
        # Checks that data has been published for each table column
        if not (self.screen_bool        
                and self.time_bool
                and self.pos_bool
                and self.force_bool):
            return
        
        # Adds a new tuple of data to pending_samples
        self.pending_samples.append((
            self.screen,
            self.curr_time,
            *self.pos,
            *self.force,
            self.sensor_values['eda'],      # latest sensor values (None -> NULL if not received)
            self.sensor_values['pulse'],
            self.sensor_values['pressure'],
            self.sensor_values['temp'],
            self.sensor_time
        ))

        # Resets the boolean checks for whether current data has been published
        self.screen_bool = False      
        self.time_bool = False
        self.pos_bool = False
        self.force_bool = False

    def save_samples(self):
        # Nothing saved if no data is present
        if not self.pending_samples:
            return

        # Commits buffered samples to the database
        with self.conn:
            self.conn.executemany(      # Reads buffered data into db as row tuples
                """
                INSERT INTO data
                    (screen, time, x, y, z, F_x, F_y, F_z,
                     eda, pulse, pressure, temp, sensor_time)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
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
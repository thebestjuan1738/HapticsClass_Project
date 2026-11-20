
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32MultiArray, String, Float32, Float64
from geometry_msgs.msg import Point

import sqlite3



class Database_Node(Node):
    def __init__(self):
        super().__init__('database_node')

        # Initialize node subscribers
        self.subscriber_pos = self.create_subscription(Float32MultiArray,'haptic_topic',self.callback_pos, qos_profile_sensor_data)
        self.subscriber_screen = self.create_subscription(String, 'interface_topic',self.callback_screen, qos_profile_sensor_data)

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
            temp REAL,
            )
        """)
        self.conn.commit()

        # Saving a published chunk of data periodically
        self.pending_samples = []   # Initializing vector of tupled samples
        self.haptic_bool = False    # Initializing boolean checks for data being published
        self.interface_bool = False
        self.sensor_bool = False
        self.save_timer = self.create_timer(0.05, self.save_samples)

    def callback_haptics(self,msg):
        self.position = msg.data[0:3]
        self.rotation = msg.data[3:6]
        self.force = msg.data[6:9]
        self.time = msg.data[9]
        self.button = bool(msg.data[10])

        self.haptic_bool = True

    def callback_interface(self,msg):
        self.curr_screen = msg.data[0]
        self.interface_bool = True

    def callback_sensor(self, name, sensor_msg):
        # Only remembers the latest value; the next row written will pick it up
        self.sensor_values[name] = sensor_msg.data
        if not (self.sensor_values['eda'] and self.sensor_values['pulse'] and self.sensor_values['temp']):
            return
        else:
            self.sensor_bool = True

    # def callback_sensor_time(self, time_msg):
    #     self.sensor_time = time_msg.data

    def buffer_sample(self):
        # Checks that data has been published for each table column
        if not (self.haptic_bool and self.interface_bool and self.sensor_bool):
            return
        
        # Adds a new tuple of data to pending_samples
        self.pending_samples.append((
            self.screen,
            self.time,
            self.button,
            *self.position,
            *self.force,
            self.sensor_values['eda'],      # latest sensor values (None -> NULL if not received)
            self.sensor_values['pulse'],
            self.sensor_values['temp'],
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
                    (screen, time, x, y, z, yaw, pitch, roll, F_x, F_y, F_z, eda, pulse, temp)
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
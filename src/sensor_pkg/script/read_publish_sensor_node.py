#!/usr/bin/env python3
import time

import numpy as np
import rclpy
import serial
from rclpy.node import Node
from sensor_pkg.msg import AllSensors, LoneSensor, SensorUserCommand


class Sensor:
    def __init__(self, sensor_id=0, fx=0, fy=0, fz=0):
        self.id = sensor_id
        self.fx = fx
        self.fy = fy
        self.fz = fz
        self.abs = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.is_present = True
        self.is_3d = True


def cart2sph(x, y, z):
    hxy = np.hypot(x, y)
    radius = np.hypot(hxy, z)
    elevation = np.pi / 2 - np.arctan2(hxy, z)
    azimuth = np.arctan2(y, x)
    return radius, elevation, azimuth


class SeedFTS3Node(Node):
    def __init__(self):
        super().__init__('seed_fts3')

        self.declare_parameter('sensor_number', 5)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('hand_polarity', 'R_')

        self.sensor_number = int(self.get_parameter('sensor_number').value)
        self.port = str(self.get_parameter('port').value)
        self.polarity = str(self.get_parameter('hand_polarity').value)

        if self.sensor_number <= 0:
            self.get_logger().warn('sensor_number must be > 0, forcing to 5')
            self.sensor_number = 5

        # With timestamp: ['@', sec, msec, fx1, fy1, fz1, ..., fxN, fyN, fzN]
        # Without timestamp: ['@', fx1, fy1, fz1, ..., fxN, fyN, fzN]
        self.expected_length_with_timestamp = self.sensor_number * 3 + 3
        self.expected_length_without_timestamp = self.sensor_number * 3 + 1
        self.baudrate = 1000000

        try:
            self.sensor_read = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.5,
                write_timeout=1,
            )
        except serial.SerialException:
            self.get_logger().error(f'Could not open serial port {self.port}')
            raise

        pub_sensor_topic = f'{self.polarity}AllSensors'
        sub_command_topic = f'{self.polarity}sensor_user_command'

        self.pub = self.create_publisher(AllSensors, pub_sensor_topic, 10)
        self.create_subscription(
            SensorUserCommand,
            sub_command_topic,
            self.sensor_user_command_callback,
            10,
        )

        self.sensors = [Sensor(i) for i in range(self.sensor_number)]

        self.get_logger().info(f'Using serial port: {self.port}')
        self.get_logger().info(f'Hand polarity: {self.polarity}')

        self._initialize_sensor_stream()

        self.timer = self.create_timer(1.0 / 50.0, self.publish_cycle)

    def _write_command(self, command):
        try:
            self.sensor_read.write(command.encode('ascii'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Failed to write command "{command.strip()}": {exc}')

    def _initialize_sensor_stream(self):
        time.sleep(0.1)
        self._write_command('calibrate\r\n')
        time.sleep(0.1)

        now = self.get_clock().now().to_msg()
        secs = str(now.sec)
        msecs = str(int(now.nanosec / 1000000))
        set_epoch_command = f'setepoch,{secs},{msecs}\r\n'
        self._write_command(set_epoch_command)

    def sensor_user_command_callback(self, msg):
        if msg.calibrate:
            self._write_command('calibrate\r\n')
            self.get_logger().info('Sensors calibrated')

        if msg.setepoch:
            secs = msg.epoch_sec
            msecs = msg.epoch_msec
            timestamp = f'{secs},{msecs}'
            set_epoch_command = f'setepoch,{secs},{msecs}\r\n'
            self._write_command(set_epoch_command)
            self.get_logger().info(f'Command sent to set epoch to {timestamp}')
            time.sleep(0.1)

        if msg.diagnosis_request:
            self._write_command('dumpsensors\r\n')
            time.sleep(0.1)
            self._write_command('getversion\r\n')
            time.sleep(0.1)
            self._write_command('softwareversion\r\n')
            time.sleep(0.1)
            self._write_command('dumponboard\r\n')
            time.sleep(0.1)
            self._write_command('resume\r\n')

        if msg.set_frequency:
            if msg.frequency <= 0:
                self.get_logger().warn('Frequency must be > 0')
            else:
                period_ms = (1.0 / msg.frequency) * 1000.0
                self.get_logger().info(f'requested period is : {period_ms}')
                self._write_command(f'setperiod,{period_ms}')
                if period_ms >= 20:
                    self.get_logger().info(f'Sensor frequency set to {msg.frequency}')
                else:
                    self.get_logger().info(
                        'Tried to change frequency above the limit. Try again with a value between 1 and 50'
                    )
                time.sleep(0.1)

        if msg.raw_string:
            self._write_command(msg.raw)
            self.get_logger().info(f'Command {msg.raw} sent')
            self.get_logger().info('raw data sent')
            time.sleep(0.1)

    def _infer_has_timestamp(self, data):
        if not data or data[0] != '@':
            return None
        if len(data) == self.expected_length_with_timestamp:
            return True
        if len(data) == self.expected_length_without_timestamp:
            return False
        self.get_logger().warn(
            f'Incomplete sensor frame. Expected {self.expected_length_with_timestamp} '
            f'or {self.expected_length_without_timestamp} fields, got {len(data)}'
        )
        return None

    def parse_data_into_obj(self, data, has_timestamp):
        if not data:
            return None
        if data[0] == '@':
            if has_timestamp:
                timestamp = f'{data[1]},{data[2]}'
                data_offset = 3
            else:
                timestamp = None
                data_offset = 1
            for i, sensor in enumerate(self.sensors):
                fx_idx = data_offset + 0 + 3 * i
                fy_idx = data_offset + 1 + 3 * i
                fz_idx = data_offset + 2 + 3 * i

                fx_raw = data[fx_idx].strip()
                fy_raw = data[fy_idx].strip()
                fz_raw = data[fz_idx].strip()

                if fz_raw == '':
                    sensor.id = i
                    sensor.fx = 0
                    sensor.fy = 0
                    sensor.fz = 0
                    sensor.is_present = False
                    continue

                sensor.is_present = True

                if fx_raw == '' or fy_raw == '':
                    sensor.id = i
                    sensor.fx = 0
                    sensor.fy = 0
                    sensor.fz = int(fz_raw)
                    sensor.is_3d = False
                    continue

                try:
                    fx = int(fx_raw)
                    fy = int(fy_raw)
                    fz = int(fz_raw)
                except ValueError:
                    self.get_logger().warn(f'Invalid sensor sample for sensor {i}: {fx_raw},{fy_raw},{fz_raw}')
                    sensor.is_present = False
                    continue

                sensor.id = i
                sensor.fx = fx
                sensor.fy = fy
                sensor.fz = fz
                sensor.is_3d = True
                radius, theta, phi = cart2sph(fx, fy, fz)
                sensor.abs = float(radius)
                sensor.pitch = float(theta)
                sensor.yaw = float(phi)
            return timestamp

        if data[0] == '#OK':
            message = ''.join(data)
            self.get_logger().info(message)
            return None

        self.get_logger().warn(f'A sensor line could not be read: wrong start character {data}')
        return None

    def parse_data(self):
        try:
            line = self.sensor_read.readline()
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            return None

        if not line:
            return None

        try:
            line_str = line.decode('utf-8').strip()
        except UnicodeDecodeError:
            self.get_logger().warn('Received undecodable bytes from serial')
            return None

        data = line_str.split(',')
        if data and data[-1] == '':
            data = data[:-1]

        if data and data[0] == '#OK':
            return self.parse_data_into_obj(data, has_timestamp=None)

        has_timestamp = self._infer_has_timestamp(data)
        if has_timestamp is None:
            return None
        return self.parse_data_into_obj(data, has_timestamp)

    def publish_cycle(self):
        _ = self.parse_data()

        filtered_sensors = [sensor for sensor in self.sensors if sensor.is_present]
        lone_sensor_msgs = []
        for sensor in filtered_sensors:
            message = LoneSensor()
            message.id = sensor.id
            message.fx = sensor.fx
            message.fy = sensor.fy
            message.fz = sensor.fz
            message.abs = sensor.abs
            message.yaw = sensor.yaw
            message.pitch = sensor.pitch
            message.is_present = sensor.is_present
            message.is_3d = sensor.is_3d
            lone_sensor_msgs.append(message)

        final_msg = AllSensors()
        final_msg.length = self.sensor_number
        final_msg.header.stamp = self.get_clock().now().to_msg()
        final_msg.data = lone_sensor_msgs
        self.pub.publish(final_msg)

    def destroy_node(self):
        if hasattr(self, 'sensor_read') and self.sensor_read and self.sensor_read.is_open:
            self.sensor_read.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SeedFTS3Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Nó ROS 2 para NAV-PVT e ESF-RAW do u-blox NEO-M8U."""

import math
import struct
import threading
import time
from typing import Dict

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from .ubx import UbxFrame, UbxStreamParser, cfg_msg, iter_esf_raw


NAV_CLASS = 0x01
NAV_PVT_ID = 0x07
ESF_CLASS = 0x10
ESF_RAW_ID = 0x03

GYRO_TYPES = {14: 'gx', 13: 'gy', 5: 'gz'}
ACCEL_TYPES = {16: 'ax', 17: 'ay', 18: 'az'}
REQUIRED_IMU_FIELDS = frozenset((*GYRO_TYPES.values(), *ACCEL_TYPES.values()))


class UbloxM8uNode(Node):
    def __init__(self) -> None:
        super().__init__('ublox_m8u')

        self.declare_parameter('device', '/dev/vida_ublox')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('gps_topic', '/vida/sensors/gps/fix')
        self.declare_parameter('imu_topic', '/vida/sensors/imu/raw')
        self.declare_parameter('configure_receiver', True)
        self.declare_parameter('reconnect_delay', 2.0)

        self._device = str(self.get_parameter('device').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._gps_frame = str(self.get_parameter('gps_frame_id').value)
        self._imu_frame = str(self.get_parameter('imu_frame_id').value)
        gps_topic = str(self.get_parameter('gps_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        self._configure_receiver = bool(self.get_parameter('configure_receiver').value)
        self._reconnect_delay = float(self.get_parameter('reconnect_delay').value)

        self._gps_pub = self.create_publisher(NavSatFix, gps_topic, 10)
        self._imu_pub = self.create_publisher(Imu, imu_topic, 100)
        self._parser = UbxStreamParser()
        self._pending_imu: Dict[int, Dict[str, float]] = {}
        self._stop_event = threading.Event()
        self._serial = None
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'NEO-M8U: aguardando {self._device}; GPS={gps_topic}; IMU={imu_topic}'
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _read_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self._serial = serial.Serial(
                    self._device,
                    self._baudrate,
                    timeout=0.2,
                    write_timeout=1.0,
                )
                self.get_logger().info(f'Conectado ao receptor em {self._device}')
                if self._configure_receiver:
                    self._enable_messages()

                while rclpy.ok() and not self._stop_event.is_set():
                    data = self._serial.read(4096)
                    if not data:
                        continue
                    for frame in self._parser.feed(data):
                        self._handle_frame(frame)
            except (serial.SerialException, OSError) as error:
                self.get_logger().error(
                    f'Falha na serial {self._device}: {error}; nova tentativa em '
                    f'{self._reconnect_delay:.1f} s',
                    throttle_duration_sec=5.0,
                )
                self._stop_event.wait(self._reconnect_delay)
            finally:
                if self._serial is not None and self._serial.is_open:
                    self._serial.close()
                self._serial = None

    def _enable_messages(self) -> None:
        # NAV-PVT: uma mensagem por época de navegação.
        self._serial.write(cfg_msg(NAV_CLASS, NAV_PVT_ID, 1))
        time.sleep(0.05)
        # ESF-RAW: qualquer taxa positiva habilita todas as amostras do sensor.
        self._serial.write(cfg_msg(ESF_CLASS, ESF_RAW_ID, 1))
        self._serial.flush()

    def _handle_frame(self, frame: UbxFrame) -> None:
        if frame.msg_class == NAV_CLASS and frame.msg_id == NAV_PVT_ID:
            self._publish_nav_pvt(frame.payload)
        elif frame.msg_class == ESF_CLASS and frame.msg_id == ESF_RAW_ID:
            self._publish_esf_raw(frame.payload)

    def _publish_nav_pvt(self, payload: bytes) -> None:
        if len(payload) < 92:
            self.get_logger().warning('UBX-NAV-PVT com tamanho inválido')
            return

        fix_type = payload[20]
        flags = payload[21]
        longitude = struct.unpack_from('<i', payload, 24)[0] * 1e-7
        latitude = struct.unpack_from('<i', payload, 28)[0] * 1e-7
        altitude = struct.unpack_from('<i', payload, 36)[0] * 1e-3
        horizontal_accuracy = struct.unpack_from('<I', payload, 40)[0] * 1e-3
        vertical_accuracy = struct.unpack_from('<I', payload, 44)[0] * 1e-3

        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._gps_frame
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = altitude

        valid_fix = bool(flags & 0x01) and fix_type in (2, 3, 4)
        message.status.status = (
            NavSatStatus.STATUS_FIX if valid_fix else NavSatStatus.STATUS_NO_FIX
        )
        message.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_GALILEO
            | NavSatStatus.SERVICE_COMPASS
        )
        message.position_covariance = [
            horizontal_accuracy ** 2, 0.0, 0.0,
            0.0, horizontal_accuracy ** 2, 0.0,
            0.0, 0.0, vertical_accuracy ** 2,
        ]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gps_pub.publish(message)

    def _publish_esf_raw(self, payload: bytes) -> None:
        for data_type, raw_value, time_tag in iter_esf_raw(payload):
            sample = self._pending_imu.setdefault(time_tag, {})
            if data_type in GYRO_TYPES:
                # 2^-12 deg/s -> rad/s.
                sample[GYRO_TYPES[data_type]] = (
                    raw_value * (2.0 ** -12) * math.pi / 180.0
                )
            elif data_type in ACCEL_TYPES:
                # 2^-10 m/s^2 (força específica).
                sample[ACCEL_TYPES[data_type]] = raw_value * (2.0 ** -10)

            if REQUIRED_IMU_FIELDS.issubset(sample):
                self._publish_imu_sample(sample)
                del self._pending_imu[time_tag]

        # Evita crescimento se uma mensagem tiver amostras incompletas.
        if len(self._pending_imu) > 100:
            for old_tag in sorted(self._pending_imu)[:-50]:
                del self._pending_imu[old_tag]

    def _publish_imu_sample(self, sample: Dict[str, float]) -> None:
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._imu_frame
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = sample['gx']
        message.angular_velocity.y = sample['gy']
        message.angular_velocity.z = sample['gz']
        message.linear_acceleration.x = sample['ax']
        message.linear_acceleration.y = sample['ay']
        message.linear_acceleration.z = sample['az']
        # Variâncias desconhecidas: matrizes mantidas em zero, conforme Imu.msg.
        self._imu_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UbloxM8uNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


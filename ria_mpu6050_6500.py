"""Raspberry Pi I2C driver for basic MPU6050 / MPU6500 measurements.

Requires: sudo apt install python3-smbus2
Supports acceleration (g) and angular rate (degrees/s).
"""

import time
from smbus2 import SMBus


class MPU6050_6500:
    WHO_AM_I = 0x75
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    ACCEL_CONFIG2 = 0x1D  # MPU6500 only
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus=1, address=0x68):
        if address not in (0x68, 0x69):
            raise ValueError("MPU I2C address must be 0x68 or 0x69")

        self.address = address
        self.bus = SMBus(bus)
        self.model = None
        self.device_id = None
        try:
            self.device_id = self.bus.read_byte_data(self.address, self.WHO_AM_I)
            if self.device_id == 0x68:
                self.model = "MPU6050-compatible"  # MPU6000 also uses ID 0x68
            elif self.device_id == 0x70:
                self.model = "MPU6500"
            else:
                raise RuntimeError(
                    f"Unsupported WHO_AM_I: 0x{self.device_id:02X}"
                )

            # Wake device; choose PLL with X gyro as clock source.
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_2, 0x00)
            time.sleep(0.1)

            # Common registers: +/- 2g, +/- 250 deg/s.
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

            # DLPF_CFG=3 in CONFIG: gyro low-pass on both chips.
            # On MPU6050 this also sets accelerometer filtering.
            self.bus.write_byte_data(self.address, self.CONFIG, 0x03)
            if self.device_id == 0x70:
                # MPU6500 uses a separate accelerometer low-pass register.
                self.bus.write_byte_data(self.address, self.ACCEL_CONFIG2, 0x03)
        except Exception:
            self.bus.close()
            raise

    @staticmethod
    def _signed_word(high, low):
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def read_all(self):
        """Read accel and gyro in one 14-byte I2C transaction.

        Returns: (ax, ay, az, gx, gy, gz)
        Units: accel=g, gyro=degrees/s
        """
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)
        accel = [self._signed_word(data[i], data[i + 1]) / 16384.0
                 for i in (0, 2, 4)]
        gyro = [self._signed_word(data[i], data[i + 1]) / 131.0
                for i in (8, 10, 12)]
        return (*accel, *gyro)

    def read_accel(self):
        """Return (ax, ay, az) in g."""
        return self.read_all()[:3]

    def read_gyro(self):
        """Return (gx, gy, gz) in degrees/s."""
        return self.read_all()[3:]

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None


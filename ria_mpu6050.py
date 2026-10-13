# ria_mpu6050.py

from smbus2 import SMBus
import time


class MPU6050:
    ADDR = 0x68

    # MPU6050 레지스터 주소
    PWR_MGMT_1 = 0x6B
    WHO_AM_I = 0x75
    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def __init__(self, bus=1, address=0x68):
        self.address = address
        self.bus = SMBus(bus)

        try:
            # 센서 확인
            device_id = self.bus.read_byte_data(
                self.address, self.WHO_AM_I
            )

            if device_id != 0x68:
                raise RuntimeError(
                    f"MPU6050 확인 실패: WHO_AM_I=0x{device_id:02X}"
                )

            # 절전 모드 해제
            self.bus.write_byte_data(
                self.address, self.PWR_MGMT_1, 0x00
            )

            # 가속도 측정 범위: ±2g
            self.bus.write_byte_data(
                self.address, self.ACCEL_CONFIG, 0x00
            )

            # 자이로 측정 범위: ±250 deg/s
            self.bus.write_byte_data(
                self.address, self.GYRO_CONFIG, 0x00
            )

            time.sleep(0.1)

        except Exception:
            self.bus.close()
            raise

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)

        value = (high << 8) | low

        # 16비트 signed 정수 변환
        if value >= 0x8000:
            value -= 65536

        return value

    def read_accel(self):
        """3축 가속도 읽기 (단위: g)"""
        x = self._read_word(self.ACCEL_XOUT_H)
        y = self._read_word(self.ACCEL_XOUT_H + 2)
        z = self._read_word(self.ACCEL_XOUT_H + 4)

        return (x / 16384.0,
                y / 16384.0,
                z / 16384.0)

    def read_gyro(self):
        """3축 각속도 읽기 (단위: deg/s)"""
        x = self._read_word(self.GYRO_XOUT_H)
        y = self._read_word(self.GYRO_XOUT_H + 2)
        z = self._read_word(self.GYRO_XOUT_H + 4)

        return (x / 131.0,
                y / 131.0,
                z / 131.0)

    def close(self):
        self.bus.close()

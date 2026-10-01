"""Riatech HTU21D 온습도 센서 모듈. Raspberry Pi의 I2C를 사용합니다."""

from time import sleep

from smbus2 import SMBus, i2c_msg


class HTU21D:
    """channel: I2C 버스 번호, address: 센서의 7비트 I2C 주소."""

    def __init__(self, channel=1, address=0x40):
        self._address = address
        self._bus = SMBus(channel)
        try:
            self._bus.write_byte(self._address, 0xFE)
            sleep(0.05)
        except BaseException:
            self.close()
            raise

    def _read_raw(self, command):
        if self._bus is None:
            raise RuntimeError("센서 연결이 이미 종료되었습니다.")

        self._bus.write_byte(self._address, command)
        sleep(0.1)

        # 결과 2바이트만 읽습니다. 이 기초 실습 모듈은 CRC 검사를 생략합니다.
        message = i2c_msg.read(self._address, 2)
        self._bus.i2c_rdwr(message)
        data = list(message)

        # 하위 2비트는 측정값이 아닌 상태 정보입니다.
        return ((data[0] << 8) | data[1]) & 0xFFFC

    def read_temperature(self):
        """온도를 새로 측정하여 섭씨(°C) 실수로 반환합니다."""
        raw = self._read_raw(0xF3)
        return -46.85 + 175.72 * raw / 65536

    def read_humidity(self):
        """상대습도를 새로 측정하여 %RH 실수로 반환합니다."""
        raw = self._read_raw(0xF5)
        return -6.0 + 125.0 * raw / 65536

    def read_temperature_humidity(self):
        """온도와 습도를 순서대로 측정하여 (온도, 습도) 튜플로 반환합니다."""
        return self.read_temperature(), self.read_humidity()

    def close(self):
        """I2C 연결을 종료합니다. 여러 번 호출해도 됩니다."""
        if self._bus is not None:
            self._bus.close()
            self._bus = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


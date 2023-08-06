from machine import Pin, I2C
import utime
import ina3221
from ina3221 import INA3221


class Tester:
    def __init__(self, i2c_id, scl: Pin, sda: Pin, i2c_freq: int):
        self.led = Pin(25, Pin.OUT)
        self.i2c = I2C(i2c_id, scl=scl, sda=sda, freq=i2c_freq)
        self._scanI2C()
        self.current_sensor = INA3221(
            self.i2c,
            c_averaging_samples=ina3221.C_AVERAGING_128_SAMPLES,
        )

        self.current_sensor.enable_channel(1, True)
        self.current_sensor.enable_channel(2, False)
        self.current_sensor.enable_channel(3, False)
        print("Waiting for current sensor to be ready")
        while not self.current_sensor.is_ready:
            utime.sleep(0.1)

    def _scanI2C(self):
        i2c_devices = self.i2c.scan()
        for device_addr in i2c_devices:
            print(
                "Found I2C device at : " + hex(device_addr).upper()
            )  # Display device address
        return

    def main(self):
        print("Starting")

        while True:
            utime.sleep(1)
            self.led.toggle()
            current_ch1 = self.current_sensor.current(1)
            current_ch2 = self.current_sensor.current(2)
            voltage_ch1 = self.current_sensor.bus_voltage(1)
            voltage_ch2 = self.current_sensor.bus_voltage(2)
            print(
                f"Current at channel 1: {current_ch1} A\nCurrent at channel 2: {current_ch2} A"
            )
            print(
                f"Voltage at channel 1: {voltage_ch1} V\nVoltage at channel 2: {voltage_ch2} V\n"
            )


if __name__ == "__main__":
    """
    Flashes LED and reads data from I2C device 0 with SDA/SCL pins on GPIO20/GPIO21
    """
    tester = Tester(
        i2c_id=0, scl=Pin(21), sda=Pin(20), i2c_freq=400000)
    tester.main()

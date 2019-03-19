# The MIT License (MIT)
#
# Copyright (c) 2019 Barbudor (IRL Jean-Michel Mercier)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`barbudor_INA3221`
====================================================

CircuitPython driver for the Texas Instruments' INA3221 3 channels current sensor.
Product page : http://www.ti.com/product/INA3221

* Author : Barbudor (IRL Jean-Michel Mercier)

Implementation Notes
--------------------

**Important Note**
In order to be coherent with the datasheet, the channel index in the below interface is 1-based
Value of ``channel`` parameter must be ``1``, ``2`` or ``3``. __Do_not_use_``0``__

Memory usage (tested with CircuitPython 4.0.0beta5 on CircuitPlayground Express):
    from barbudor_ina3221 import INA3221        --> 6592 bytes
    ina3221 = INA3221(i2c_bus)                  -->  112 bytes

**Hardware:**

* `INA3221 Triple, Low-/High-Side, I2C Out Current/Voltage Monitor :
    <http://www.ti.com/product/INA3221>`_

* Available breakouts :
- `SwitchDoc Labs :
    <https://shop.switchdoc.com/collections/sensors/products/ina3221-breakout-board-3-channel-current-voltage-monitor-grove-headers-compare-to-ina219-grove-headers>`_
- Generic chinese INA3221 breakout available on eBay or AliExpress

**Software and Dependencies:**

* Adafruit CircuitPython firmware (3.1+):  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice


# pylint: disable=bad-whitespace

_DEFAULT_ADDRESS                 = const(0x40)

#
# Registers and bits definitions
#

# Config register
C_REG_CONFIG                      = const(0x00)

C_RESET                           = const(0x8000)
C_ENABLE_CH1                      = const(0x4000)     # default set
C_ENABLE_CH2                      = const(0x2000)     # default set
C_ENABLE_CH3                      = const(0x1000)     # default set

C_AVERAGING_MASK                  = const(0x0E00)
C_AVERAGING_NONE                  = const(0x0000)     # 1 sample, default
C_AVERAGING_4_SAMPLES             = const(0x0200)
C_AVERAGING_16_SAMPLES            = const(0x0400)
C_AVERAGING_64_SAMPLES            = const(0x0600)
C_AVERAGING_128_SAMPLES           = const(0x0800)
C_AVERAGING_256_SAMPLES           = const(0x0A00)
C_AVERAGING_512_SAMPLES           = const(0x0C00)
C_AVERAGING_1024_SAMPLES          = const(0x0E00)

C_VBUS_CONV_TIME_MASK             = const(0x01C0)
C_VBUS_CONV_TIME_140US            = const(0x0000)
C_VBUS_CONV_TIME_204US            = const(0x0040)
C_VBUS_CONV_TIME_332US            = const(0x0080)
C_VBUS_CONV_TIME_588US            = const(0x00C0)
C_VBUS_CONV_TIME_1MS              = const(0x0100)     # 1.1ms, default
C_VBUS_CONV_TIME_2MS              = const(0x0140)     # 2.116ms
C_VBUS_CONV_TIME_4MS              = const(0x0180)     # 4.156ms
C_VBUS_CONV_TIME_8MS              = const(0x01C0)     # 8.244ms

C_SHUNT_CONV_TIME_MASK            = const(0x0038)
C_SHUNT_CONV_TIME_140US           = const(0x0000)
C_SHUNT_CONV_TIME_204US           = const(0x0008)
C_SHUNT_CONV_TIME_332US           = const(0x0010)
C_SHUNT_CONV_TIME_588US           = const(0x0018)
C_SHUNT_CONV_TIME_1MS             = const(0x0020)     # 1.1ms, default
C_SHUNT_CONV_TIME_2MS             = const(0x0028)     # 2.116ms
C_SHUNT_CONV_TIME_4MS             = const(0x0030)     # 4.156ms
C_SHUNT_CONV_TIME_8MS             = const(0x0038)     # 8.244ms

C_MODE_MASK                       = const(0x0007)
C_MODE_POWER_DOWN                 = const(0x0000)     # Power-down
C_MODE_SHUNT_VOLTAGE_TRIGGERED    = const(0x0001)     # Shunt voltage, single-shot (triggered)
C_MODE_BUS_VOLTAGE_TRIGGERED      = const(0x0002)     # Bus voltage, single-shot (triggered)
C_MODE_SHUNT_AND_BUS_TRIGGERED    = const(0x0003)     # Shunt and bus, single-shot (triggered)
C_MODE_POWER_DOWN2                = const(0x0004)     # Power-down
C_MODE_SHUNT_VOLTAGE_CONTINUOUS   = const(0x0005)     # Shunt voltage, continous
C_MODE_BUS_VOLTAGE_CONTINUOUS     = const(0x0006)     # Bus voltage, continuous
C_MODE_SHUNT_AND_BUS_CONTINOUS    = const(0x0007)     # Shunt and bus, continuous (default)

# Other registers
C_REG_SHUNT_VOLTAGE_CH1           = const(0x01)
C_REG_BUS_VOLTAGE_CH1             = const(0x02)
C_REG_SHUNT_VOLTAGE_CH2           = const(0x03)
C_REG_BUS_VOLTAGE_CH2             = const(0x04)
C_REG_SHUNT_VOLTAGE_CH3           = const(0x05)
C_REG_BUS_VOLTAGE_CH3             = const(0x06)
C_REG_CRITICAL_ALERT_LIMIT_CH1    = const(0x07)
C_REG_WARNING_ALERT_LIMIT_CH1     = const(0x08)
C_REG_CRITICAL_ALERT_LIMIT_CH2    = const(0x09)
C_REG_WARNING_ALERT_LIMIT_CH2     = const(0x0A)
C_REG_CRITICAL_ALERT_LIMIT_CH3    = const(0x0B)
C_REG_WARNING_ALERT_LIMIT_CH3     = const(0x0C)
C_REG_SHUNT_VOLTAGE_SUM           = const(0x0D)
C_REG_SHUNT_VOLTAGE_SUM_LIMIT     = const(0x0E)

# Mask/enable register
C_REG_MASK_ENABLE                 = const(0x0F)
C_SUM_CONTROL_CH1                 = const(0x4000)     # default not set
C_SUM_CONTROL_CH2                 = const(0x2000)     # default not set
C_SUM_CONTROL_CH3                 = const(0x1000)     # default not set
C_WARNING_LATCH_ENABLE            = const(0x0800)     # default not set
C_CRITICAL_LATCH_ENABLE           = const(0x0400)     # default not set
C_CRITICAL_FLAG_CH1               = const(0x0200)
C_CRITICAL_FLAG_CH2               = const(0x0100)
C_CRITICAL_FLAG_CH3               = const(0x0080)
C_SUM_ALERT_FLAG                  = const(0x0040)
C_WARNING_FLAG_CH1                = const(0x0020)
C_WARNING_FLAG_CH2                = const(0x0010)
C_WARNING_FLAG_CH3                = const(0x0008)
C_POWER_ALERT_FLAG                = const(0x0004)
C_TIMING_ALERT_FLAG               = const(0x0002)
C_CONV_READY_FLAG                 = const(0x0001)

# Other registers
C_REG_POWER_VALID_UPPER_LIMIT     = const(0x10)
C_REG_POWER_VALID_LOWER_LIMIT     = const(0x11)
C_REG_MANUFACTURER_ID             = const(0xFE)
C_REG_DIE_ID                      = const(0xFF)

# Constants for manufacturer and device ID
C_MANUFACTURER_ID                 = const(0x5449)     # "TI"
C_DIE_ID                          = const(0x3220)


class INA3221:
    """Driver class for Texas Instruments INA3221 3 channel current sensor device"""

    @staticmethod
    def _to_signed(val):
        if val > 32767:
            return val - 65536
        return val

    @staticmethod
    def _to_unsigned(val):
        if val < 0:
            return val + 65536
        return val

    def write(self, reg, value):
        """Write value in device register"""
        seq = bytearray([reg, (value >> 8) & 0xFF, value & 0xFF])
        with self.i2c_device as i2c:
            i2c.write(seq)

    def read(self, reg):
        """Return value from device register"""
        buf = bytearray(3)
        buf[0] = reg
        with self.i2c_device as i2c:
            i2c.write(buf, end=1, stop=False)
            i2c.readinto(buf, start=1)
        value = (buf[1] << 8) | (buf[2])
        return value

    def update(self, reg, mask, value):
        """Read-modify-write value in register"""
        regvalue = self.read(reg)
        regvalue &= ~mask
        value &= mask
        self.write(reg, regvalue | value)


    def __init__(self, i2c_bus, i2c_addr = _DEFAULT_ADDRESS, shunt_resistor = (0.1, 0.1, 0.1)):
        self.i2c_device = I2CDevice(i2c_bus, i2c_addr)
        self.i2c_addr = i2c_addr
        self.shunt_resistor = shunt_resistor

        self.write(C_REG_CONFIG,  C_AVERAGING_16_SAMPLES | \
                                  C_VBUS_CONV_TIME_1MS | \
                                  C_SHUNT_CONV_TIME_1MS | \
                                  C_MODE_SHUNT_AND_BUS_CONTINOUS )

    def is_channel_enabled(self, channel=1):
        """Returns is agiven channel is enabled or not"""
        bit = C_ENABLE_CH1 >> (channel-1)
        return self.read(C_REG_CONFIG) & bit != 0

    def enable_channel(self, channel=1, enable=True):
        """Enables or disable a given channel"""
        bit = C_ENABLE_CH1 >> (channel-1)
        value = 0
        if enable:
            value = bit
        self.update(C_REG_CONFIG, bit, value)

    def shunt_voltage(self, channel=1):
        """Returns the channel's shunt voltage in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_signed(self.read(C_REG_SHUNT_VOLTAGE_CH1 + channel-1)) / 8.0
        # convert to volts - LSB = 40uV
        return value * 0.00004

    def current(self, channel=1):
        """Return's the channel current in A"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        return self.shunt_voltage(channel) / self.shunt_resistor[channel-1]

    def bus_voltage(self, channel=1):
        """Returns the channel's bus voltage in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_signed(self.read(C_REG_BUS_VOLTAGE_CH1 + channel-1)) / 8
        # convert to volts - LSB = 8mV
        return value * 0.008

    def shunt_critical_alert_limit(self, channel=1):
        """Returns the channel's shunt voltage critical alert limit in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_signed(self.read(C_REG_CRITICAL_ALERT_LIMIT_CH1 + channel-1)) / 8
        # convert to volts - LSB = 40uV
        return value * 0.00004

    def set_shunt_critical_alert_limit(self, channel, voltage):
        """Sets the channel's shunt voltage critical alert limit in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_unsigned(round(voltage * 0.00004) * 8)
        self.write(C_REG_CRITICAL_ALERT_LIMIT_CH1 + channel-1, value)

    def shunt_warning_alert_limit(self, channel=1):
        """Returns the channel's shunt voltage warning alert limit in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_signed(self.read(C_REG_WARNING_ALERT_LIMIT_CH1 + channel-1)) / 8
        # convert to volts - LSB = 40uV
        return value * 0.00004

    # why do I need this ???
    # pylint: disable=too-many-function-args
    def set_shunt_warning_alert_limit(self, channel, voltage):
        """Sets the channel's shunt voltage warning alert limit in Volts"""
        assert 1 <= channel <= 3, "channel argument must be 1, 2, or 3"
        value = self._to_unsigned(round(voltage * 0.00004) * 8)
        self.read(C_REG_WARNING_ALERT_LIMIT_CH1 + channel-1, value)
    # pylint: enable=too-many-function-args
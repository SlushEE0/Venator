# SPDX-FileCopyrightText: 2016 Scott Shawcroft for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bus_device.i2c_device` - I2C Bus Device
====================================================
"""

import time

try:
    from typing import Optional, Type
    from types import TracebackType
    from circuitpython_typing import ReadableBuffer, WriteableBuffer

    # Used only for type annotations.
    from busio import I2C
except ImportError:
    pass


__version__ = "5.2.10"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BusDevice.git"


class I2CDevice:
    """
    Represents a single I2C device and manages locking the bus and the device
    address for MicroPython's machine.I2C implementation.
    """

    def __init__(self, i2c, device_address: int, probe: bool = True) -> None:
        self.i2c = i2c
        self.device_address = device_address

        if probe:
            self.__probe_for_device()

    def readinto(self, buf, *, start: int = 0, end: int = None) -> None:
        """
        Read into `buf` from the device. The number of bytes read will be the length of `buf`.

        :param buf: buffer to write into
        :param start: Index to start writing at
        :param end: Index to write up to but not include; if None, use `len(buf)`
        """
        if end is None:
            end = len(buf)
        self.i2c.readfrom_into(self.device_address, buf[start:end])

    def write(self, buf, *, start: int = 0, end: int = None) -> None:
        """
        Write the bytes from `buf` to the device.

        :param buf: buffer containing the bytes to write
        :param start: Index to start writing from
        :param end: Index to write up to but not include; if None, use `len(buf)`
        """
        if end is None:
            end = len(buf)
        self.i2c.writeto(self.device_address, buf[start:end])

    def write_then_readinto(
        self,
        out_buffer,
        in_buffer,
        *,
        out_start: int = 0,
        out_end: int = None,
        in_start: int = 0,
        in_end: int = None,
    ) -> None:
        """
        Write the bytes from `out_buffer` to the device, then immediately reads
        into `in_buffer` from the device.

        :param out_buffer: buffer containing the bytes to write
        :param in_buffer: buffer to read into
        :param out_start: Index to start writing from
        :param out_end: Index to write up to but not include; if None, use `len(out_buffer)`
        :param in_start: Index to start reading into
        :param in_end: Index to read up to but not include; if None, use `len(in_buffer)`
        """
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)
        self.i2c.writeto_then_readfrom(
            self.device_address,
            out_buffer[out_start:out_end],
            in_buffer[in_start:in_end],
        )

    def __probe_for_device(self) -> None:
        """
        Probe for the device to confirm it exists at the specified address.
        """
        try:
            self.i2c.writeto(self.device_address, b"")
        except OSError:
            try:
                result = bytearray(1)
                self.i2c.readfrom_into(self.device_address, result)
            except OSError:
                raise ValueError(f"No I2C device found at address: 0x{self.device_address:02X}")


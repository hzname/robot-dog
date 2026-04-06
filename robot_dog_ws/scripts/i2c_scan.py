#!/usr/bin/env python3
"""Scan all I2C buses for PCA9685"""
import smbus2

for bus_num in [5, 6]:
    for addr in [0x30, 0x40, 0x70, 0x60, 0x36]:
        try:
            bus = smbus2.SMBus(bus_num)
            bus.write_byte_data(addr, 0x00, 0x00)
            print(f"bus={bus_num} addr=0x{addr:02x} WRITE OK")
            bus.close()
        except Exception as e:
            print(f"bus={bus_num} addr=0x{addr:02x} FAIL: {type(e).__name__}: {e}")

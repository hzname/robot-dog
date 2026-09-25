#!/usr/bin/env python3
"""Scan all I2C buses for PCA9685 and other devices."""
import smbus2

# Banana Pi: bus 0 (I2C0), bus 5, bus 6
BUSES = [0, 5, 6]
ADDRESSES = [0x30, 0x40, 0x60, 0x70]

for bus_num in BUSES:
    print(f"\n--- Scanning I2C bus {bus_num} ---")
    try:
        bus = smbus2.SMBus(bus_num)
    except Exception as e:
        print(f"  Cannot open bus {bus_num}: {e}")
        continue
    
    for addr in range(0x03, 0x78):
        try:
            bus.write_byte_data(addr, 0x00, 0x00)
            label = ""
            if addr == 0x40:
                label = " (PCA9685 default)"
            elif addr == 0x30:
                label = " (PCA9685 alt)"
            print(f"  ✓ 0x{addr:02X}{label}")
        except:
            pass
    bus.close()
    
print("\nDone.")

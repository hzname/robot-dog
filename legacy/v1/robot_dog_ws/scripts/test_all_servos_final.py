"""Test all 12 servos with CORRECT byte order: ON_L, ON_H, OFF_L, OFF_H"""
import smbus2
import time

bus = smbus2.SMBus(0)
addr = 0x40

# Init
bus.write_byte_data(addr, 0x00, 0x80)
time.sleep(0.01)
bus.write_byte_data(addr, 0xFE, 121)
time.sleep(0.01)
bus.write_byte_data(addr, 0x01, 0x04)
bus.write_byte_data(addr, 0x00, 0x21)
time.sleep(0.1)

print("ROBOT DOG - ALL 12 SERVOS TEST")
print("CORRECT BYTE ORDER: ON_L, ON_H, OFF_L, OFF_H\n")

names = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
         "BL_hip", "BL_thigh", "BL_calf", "BR_hip", "BR_thigh", "BR_calf"]

for ch in range(12):
    reg = 0x06 + 4 * ch
    print(f"Channel {ch} ({names[ch]}):")
    
    # 1000us
    tick = 204
    data = [0, 0, tick & 0xFF, (tick >> 8) & 0x0F]
    bus.write_i2c_block_data(addr, reg, data)
    time.sleep(0.5)
    
    # 2000us
    tick = 409
    data = [0, 0, tick & 0xFF, (tick >> 8) & 0x0F]
    bus.write_i2c_block_data(addr, reg, data)
    time.sleep(0.5)
    
    # 1500us
    tick = 307
    data = [0, 0, tick & 0xFF, (tick >> 8) & 0x0F]
    bus.write_i2c_block_data(addr, reg, data)
    time.sleep(0.3)

bus.close()
print("\nTEST COMPLETE")

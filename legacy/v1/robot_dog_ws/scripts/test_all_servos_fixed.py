"""Test all 12 servos with fixed PCA9685 driver."""
import smbus2
import time

class PCA9685:
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus_number=0, address=0x40, frequency=50):
        self.address = address
        self.frequency = frequency
        self.bus = smbus2.SMBus(bus_number)
        self._init_chip()
    
    def _init_chip(self):
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        time.sleep(0.01)
        prescale = int(round(25000000.0 / (4096.0 * self.frequency) - 1.0))
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode & 0x7F)
        time.sleep(0.01)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0x80)
        time.sleep(0.05)
        self.bus.write_byte_data(self.address, self.MODE2, 0x04)
        self.bus.write_byte_data(self.address, self.MODE1, 0x21)  # AI + ALLCALL
        time.sleep(0.01)
    
    def set_pwm(self, channel, on, off):
        base = self.LED0_ON_L + 4 * channel
        self.bus.write_i2c_block_data(
            self.address, base,
            [off & 0xFF, (off >> 8) & 0xFF, on & 0xFF, (on >> 8) & 0xFF]
        )
    
    def set_servo_pulse(self, channel, pulse_us):
        tick = int(pulse_us / 20000.0 * 4096.0)
        tick = max(0, min(4095, tick))
        self.set_pwm(channel, 0, tick)

# Test
print("="*60)
print("ROBOT DOG - ALL 12 SERVOS TEST")
print("="*60)

pca = PCA9685()
names = ["FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
         "BL_hip", "BL_thigh", "BL_calf", "BR_hip", "BR_thigh", "BR_calf"]

for ch in range(12):
    print(f"\nChannel {ch} ({names[ch]}):")
    for us in [1000, 1500, 2000]:
        pca.set_servo_pulse(ch, us)
        print(f"  {us}us")
        time.sleep(0.5)
    pca.set_servo_pulse(ch, 1500)

print("\n" + "="*60)
print("TEST COMPLETE - All servos at neutral (1500us)")
print("="*60)

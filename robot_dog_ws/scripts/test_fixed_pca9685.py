"""Test fixed PCA9685 driver."""
import smbus2
import time

class PCA9685:
    """PCA9685 PWM controller driver via I2C (FIXED VERSION)."""
    
    MODE1 = 0x00
    MODE2 = 0x01
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    
    RESTART = 0x80
    SLEEP = 0x10
    ALLCALL = 0x01
    OUTDRV = 0x04

    def __init__(self, bus_number=0, address=0x40, frequency=50):
        self.address = address
        self.frequency = frequency
        self.bus = smbus2.SMBus(bus_number)
        self._init_chip()
    
    def _init_chip(self):
        """Initialize PCA9685 chip with AI bit enabled."""
        # Full reset
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        
        # Enter sleep for prescale
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | self.SLEEP)
        time.sleep(0.01)
        
        # Set prescale
        prescale = int(round(25000000.0 / (4096.0 * self.frequency) - 1.0))
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        time.sleep(0.01)
        
        # Wake up
        self.bus.write_byte_data(self.address, self.MODE1, old_mode & 0x7F)
        time.sleep(0.01)
        
        # Restart
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | self.RESTART)
        time.sleep(0.05)
        
        # Set MODE2
        self.bus.write_byte_data(self.address, self.MODE2, self.OUTDRV)
        
        # Set MODE1 with AUTO-INCREMENT bit
        self.bus.write_byte_data(self.address, self.MODE1, 0x20 | self.ALLCALL)
        time.sleep(0.01)
    
    def set_pwm(self, channel, on, off):
        """Set PWM with correct byte order for AI mode."""
        base = self.LED0_ON_L + 4 * channel
        # OFF_L, OFF_H, ON_L, ON_H
        self.bus.write_i2c_block_data(
            self.address,
            base,
            [off & 0xFF, (off >> 8) & 0xFF, on & 0xFF, (on >> 8) & 0xFF]
        )
    
    def set_servo_pulse(self, channel, pulse_us):
        """Set servo pulse width in microseconds."""
        tick = int(pulse_us / 20000.0 * 4096.0)
        tick = max(0, min(4095, tick))
        self.set_pwm(channel, 0, tick)

# Test
print("="*60)
print("TESTING FIXED PCA9685 DRIVER")
print("="*60)

pca = PCA9685()

print("\n[1] Channel 0 - positions test")
for us in [500, 1500, 2500, 1500]:
    pca.set_servo_pulse(0, us)
    print(f"  {us}us")
    time.sleep(2)

print("\n[2] Channel 1 - sweep test")
for i in range(101):
    us = 500 + i * 20
    pca.set_servo_pulse(1, us)
    time.sleep(0.05)

for i in range(101):
    us = 2500 - i * 20
    pca.set_servo_pulse(1, us)
    time.sleep(0.05)

pca.set_servo_pulse(1, 1500)

print("\n" + "="*60)
print("TEST COMPLETE")
print("="*60)

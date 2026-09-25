"""Quadruped gait emulation for robot dog."""
import smbus2
import time
import math

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
        self.bus.write_byte_data(self.address, self.MODE1, 0x21)
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
    
    def set_all_off(self):
        for ch in range(12):
            self.set_servo_pulse(ch, 0)

class RobotLeg:
    """Single leg controller."""
    
    def __init__(self, pca, hip_ch, thigh_ch, calf_ch, inverted=False):
        self.pca = pca
        self.hip_ch = hip_ch
        self.thigh_ch = thigh_ch
        self.calf_ch = calf_ch
        self.inverted = inverted
    
    def set_position(self, hip_deg, thigh_deg, calf_deg):
        """Set all 3 joint angles."""
        if self.inverted:
            hip_deg = -hip_deg
            thigh_deg = -thigh_deg
            calf_deg = -calf_deg
        
        # Convert degrees to pulse (1000-2000us range, 1500us = 0 deg)
        def deg_to_pulse(deg):
            return 1500 + deg * 5.56  # ~5.56us per degree
        
        self.pca.set_servo_pulse(self.hip_ch, deg_to_pulse(hip_deg))
        self.pca.set_servo_pulse(self.thigh_ch, deg_to_pulse(thigh_deg))
        self.pca.set_servo_pulse(self.calf_ch, deg_to_pulse(calf_deg))

def trot_gait(pca, cycles=4, speed=0.3):
    """
    Trot gait - diagonal legs move together.
    FL+BR step together, FR+BL step together.
    """
    print("="*60)
    print("TROT GAIT - Diagonal stepping")
    print("="*60)
    
    # Initialize legs
    fl = RobotLeg(pca, 0, 1, 2, inverted=False)
    fr = RobotLeg(pca, 3, 4, 5, inverted=True)
    bl = RobotLeg(pca, 6, 7, 8, inverted=False)
    br = RobotLeg(pca, 9, 10, 11, inverted=True)
    
    # Neutral position
    neutral = (0, 0, 0)
    
    # Step pattern: lift, forward, down, back
    step_height = 20
    step_length = 30
    
    for cycle in range(cycles):
        print(f"\nCycle {cycle + 1}/{cycles}")
        
        # Phase 1: FL+BR lift and forward, FR+BL back
        print("  Phase 1: FL+BR forward, FR+BL back")
        for i in range(11):
            t = i / 10.0
            
            # FL+BR (diagonal 1)
            fl.set_position(-step_length * t, step_height * math.sin(t * math.pi), 0)
            br.set_position(-step_length * t, step_height * math.sin(t * math.pi), 0)
            
            # FR+BL (diagonal 2) - grounded, moving back
            fr.set_position(step_length * (1-t), 0, 0)
            bl.set_position(step_length * (1-t), 0, 0)
            
            time.sleep(speed / 10.0)
        
        # Phase 2: FR+BL lift and forward, FL+BR back
        print("  Phase 2: FR+BL forward, FL+BR back")
        for i in range(11):
            t = i / 10.0
            
            # FR+BL (diagonal 2)
            fr.set_position(-step_length * t, step_height * math.sin(t * math.pi), 0)
            bl.set_position(-step_length * t, step_height * math.sin(t * math.pi), 0)
            
            # FL+BR (diagonal 1) - grounded, moving back
            fl.set_position(step_length * (1-t), 0, 0)
            br.set_position(step_length * (1-t), 0, 0)
            
            time.sleep(speed / 10.0)
    
    # Return to neutral
    print("\nReturning to neutral...")
    fl.set_position(*neutral)
    fr.set_position(*neutral)
    bl.set_position(*neutral)
    br.set_position(*neutral)

def stand(pca):
    """Make robot stand in neutral position."""
    print("Standing up...")
    fl = RobotLeg(pca, 0, 1, 2, inverted=False)
    fr = RobotLeg(pca, 3, 4, 5, inverted=True)
    bl = RobotLeg(pca, 6, 7, 8, inverted=False)
    br = RobotLeg(pca, 9, 10, 11, inverted=True)
    
    neutral = (0, 0, 0)
    fl.set_position(*neutral)
    fr.set_position(*neutral)
    bl.set_position(*neutral)
    br.set_position(*neutral)
    time.sleep(1)
    print("Standing!")

def main():
    pca = PCA9685()
    
    print("="*60)
    print("ROBOT DOG - GAIT EMULATION")
    print("="*60)
    
    # Stand
    stand(pca)
    time.sleep(1)
    
    # Trot gait
    trot_gait(pca, cycles=4, speed=0.2)
    
    # Stand
    stand(pca)
    
    print("\n" + "="*60)
    print("GAIT COMPLETE")
    print("="*60)

if __name__ == '__main__':
    main()

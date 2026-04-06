/**
 * PCA9685 Test — Direct I2C access via ioctl
 * Tests channels 0 and 1 with PWM sweep
 */

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"

#define MODE1     0x00
#define MODE2     0x01
#define PRESCALE  0xFE
#define LED0_ON_L 0x06
#define RESTART   0x80
#define SLEEP     0x10
#define ALLCALL   0x01
#define OUTDRV    0x04

class PCA9685 {
private:
    int fd;
    uint8_t addr;

    void writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        if (write(fd, buf, 2) != 2) {
            throw std::runtime_error("Failed to write register");
        }
    }

    uint8_t readReg(uint8_t reg) {
        uint8_t val = reg;
        if (write(fd, &val, 1) != 1) {
            throw std::runtime_error("Failed to set register address");
        }
        if (read(fd, &val, 1) != 1) {
            throw std::runtime_error("Failed to read register");
        }
        return val;
    }

public:
    PCA9685(const std::string& bus, uint8_t address = PCA9685_ADDR) : addr(address) {
        fd = open(bus.c_str(), O_RDWR);
        if (fd < 0) {
            throw std::runtime_error("Failed to open I2C bus");
        }
        if (ioctl(fd, I2C_SLAVE, addr) < 0) {
            close(fd);
            throw std::runtime_error("Failed to set I2C slave address");
        }
    }

    ~PCA9685() {
        if (fd >= 0) close(fd);
    }

    void init(int freq = 50) {
        std::cout << "[INIT] Starting PCA9685 initialization...\n";
        writeReg(MODE1, 0x00);
        usleep(10000);
        uint8_t old_mode = readReg(MODE1);
        writeReg(MODE1, (old_mode & 0x7F) | SLEEP);
        usleep(10000);
        int prescale = (int)(25000000.0 / (4096.0 * freq) - 1.0);
        writeReg(PRESCALE, prescale);
        std::cout << "[INIT] Prescale: " << prescale << " (for " << freq << "Hz)\n";
        usleep(10000);
        writeReg(MODE1, old_mode & 0x7F);
        usleep(10000);
        writeReg(MODE1, old_mode | RESTART);
        usleep(50000);
        writeReg(MODE2, OUTDRV);
        writeReg(MODE1, ALLCALL);
        usleep(10000);
        std::cout << "[INIT] PCA9685 initialized\n";
    }

    void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
        if (channel > 15) return;
        uint8_t buf[5];
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xFF;
        buf[2] = (on >> 8) & 0xFF;
        buf[3] = off & 0xFF;
        buf[4] = (off >> 8) & 0xFF;
        write(fd, buf, 5);
    }

    void setServoPulse(uint8_t channel, float pulse_us) {
        uint16_t tick = (uint16_t)(pulse_us / 20000.0f * 4096.0f);
        if (tick > 4095) tick = 4095;
        setPWM(channel, 0, tick);
    }

    void allOff() {
        for (int i = 0; i < 16; i++) {
            setPWM(i, 0, 0);
        }
    }
};

void testChannel(PCA9685& pca, int channel, const std::string& name) {
    std::cout << "\n=== Channel " << channel << " (" << name << ") ===\n";
    
    std::cout << "[1] Neutral (1500us)\n";
    pca.setServoPulse(channel, 1500.0f);
    sleep(2);
    
    std::cout << "[2] Min (500us)\n";
    pca.setServoPulse(channel, 500.0f);
    sleep(2);
    
    std::cout << "[3] Max (2500us)\n";
    pca.setServoPulse(channel, 2500.0f);
    sleep(2);
    
    std::cout << "[4] Back to neutral\n";
    pca.setServoPulse(channel, 1500.0f);
    sleep(2);
    
    std::cout << "[5] Sweep 1000-2000us\n";
    for (int i = 0; i <= 100; i++) {
        pca.setServoPulse(channel, 1000.0f + i * 10.0f);
        usleep(50000);
    }
    for (int i = 0; i <= 100; i++) {
        pca.setServoPulse(channel, 2000.0f - i * 10.0f);
        usleep(50000);
    }
    
    std::cout << "[6] Raw ticks: 205 -> 307 -> 410 -> 0\n";
    pca.setPWM(channel, 0, 205);
    sleep(2);
    pca.setPWM(channel, 0, 307);
    sleep(2);
    pca.setPWM(channel, 0, 410);
    sleep(2);
    pca.setPWM(channel, 0, 0);
    sleep(1);
    
    std::cout << "=== Test complete ===\n";
}

int main() {
    std::cout << "PCA9685 Test (C++ / Raw I2C)\n";
    try {
        PCA9685 pca(I2C_BUS);
        pca.init(50);
        testChannel(pca, 0, "FL_hip");
        testChannel(pca, 1, "FL_thigh");
        pca.allOff();
        std::cout << "\nAll tests complete!\n";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }
    return 0;
}

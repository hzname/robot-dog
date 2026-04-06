/**
 * Standalone C++ test for PCA9685 with AI bit and correct byte order.
 */
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"

// PCA9685 registers
#define MODE1       0x00
#define MODE2       0x01
#define PRESCALE    0xFE
#define LED0_ON_L   0x06
#define ALL_LED_ON_L 0xFA

// Bits
#define RESTART     0x80
#define SLEEP       0x10
#define ALLCALL     0x01
#define OUTDRV      0x04
#define AI          0x20

class PCA9685 {
private:
    int fd;
    uint8_t addr;
    double pwm_freq = 50.0;

    bool writeByte(uint8_t reg, uint8_t data) {
        uint8_t buf[2] = {reg, data};
        return write(fd, buf, 2) == 2;
    }

    bool readByte(uint8_t reg, uint8_t& data) {
        if (write(fd, &reg, 1) != 1) return false;
        return read(fd, &data, 1) == 1;
    }

public:
    PCA9685(const std::string& bus, uint8_t address = PCA9685_ADDR) : addr(address) {
        fd = open(bus.c_str(), O_RDWR);
        if (fd < 0) {
            throw std::runtime_error("Cannot open I2C bus");
        }
        if (ioctl(fd, I2C_SLAVE, addr) < 0) {
            close(fd);
            throw std::runtime_error("Cannot set I2C slave address");
        }
        init();
    }

    ~PCA9685() {
        if (fd >= 0) close(fd);
    }

    void init() {
        std::cout << "[INIT] Initializing PCA9685 with AI bit...\n";
        
        // Reset
        writeByte(MODE1, RESTART);
        usleep(1000);
        
        // Read MODE1
        uint8_t mode1;
        readByte(MODE1, mode1);
        
        // Enable AI bit and clear sleep
        mode1 = (mode1 & ~SLEEP) | AI | ALLCALL;
        writeByte(MODE1, mode1);
        usleep(1000);
        
        // Set MODE2
        writeByte(MODE2, OUTDRV);
        usleep(10000);
        
        // Set prescale for 50Hz
        uint8_t oldmode;
        readByte(MODE1, oldmode);
        writeByte(MODE1, (oldmode & ~RESTART) | SLEEP);
        usleep(1000);
        
        uint8_t prescale = 121;
        writeByte(PRESCALE, prescale);
        usleep(1000);
        
        writeByte(MODE1, oldmode | RESTART | AI);
        usleep(10000);
        
        uint8_t newmode1, newmode2;
        readByte(MODE1, newmode1);
        readByte(MODE2, newmode2);
        
        std::cout << "[INIT] MODE1 = 0x" << std::hex << (int)newmode1 << std::dec << " (AI=" << ((newmode1>>5)&1) << ")\n";
        std::cout << "[INIT] MODE2 = 0x" << std::hex << (int)newmode2 << std::dec << "\n";
    }

    void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
        if (channel > 15) return;
        
        uint8_t reg = LED0_ON_L + 4 * channel;
        
        // FIXED: Single write with reg + data in correct order: OFF_L, OFF_H, ON_L, ON_H
        uint8_t buffer[5] = {
            reg,
            static_cast<uint8_t>(off & 0xFF),
            static_cast<uint8_t>((off >> 8) & 0x0F),
            static_cast<uint8_t>(on & 0xFF),
            static_cast<uint8_t>((on >> 8) & 0x0F)
        };
        
        write(fd, buffer, 5);
    }

    void setServoPulse(uint8_t channel, float pulse_us) {
        uint16_t tick = static_cast<uint16_t>(pulse_us / 20000.0f * 4096.0f);
        if (tick > 4095) tick = 4095;
        setPWM(channel, 0, tick);
    }
};

int main() {
    std::cout << "========================================\n";
    std::cout << "  PCA9685 C++ TEST (FIXED)\n";
    std::cout << "========================================\n";
    
    try {
        PCA9685 pca(I2C_BUS);
        
        std::cout << "\n[TEST] Channels 0 & 1 - full range\n";
        
        // Channel 0
        for (float us : {500.0f, 1500.0f, 2500.0f, 1500.0f}) {
            pca.setServoPulse(0, us);
            std::cout << "Ch0: " << us << "us\n";
            usleep(2000000);
        }
        
        // Channel 1 sweep
        std::cout << "\nCh1 sweep 500-2500us\n";
        for (int i = 0; i <= 100; i++) {
            float us = 500.0f + i * 20.0f;
            pca.setServoPulse(1, us);
            usleep(20000);
        }
        for (int i = 0; i <= 100; i++) {
            float us = 2500.0f - i * 20.0f;
            pca.setServoPulse(1, us);
            usleep(20000);
        }
        pca.setServoPulse(1, 1500.0f);
        
        std::cout << "\n========================================\n";
        std::cout << "  TEST COMPLETE\n";
        std::cout << "========================================\n";
        
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

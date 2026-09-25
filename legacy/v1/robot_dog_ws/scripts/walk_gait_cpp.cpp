/**
 * Quadruped gait emulation in C++ (standalone, no ROS2).
 * Uses fixed PCA9685 driver.
 */
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"

#define MODE1       0x00
#define MODE2       0x01
#define PRESCALE    0xFE
#define LED0_ON_L   0x06

#define RESTART     0x80
#define SLEEP       0x10
#define ALLCALL     0x01
#define OUTDRV      0x04
#define AI          0x20

class PCA9685 {
private:
    int fd;
    
    bool writeByte(uint8_t reg, uint8_t data) {
        uint8_t buf[2] = {reg, data};
        return write(fd, buf, 2) == 2;
    }

public:
    PCA9685() {
        fd = open(I2C_BUS, O_RDWR);
        if (fd < 0) throw std::runtime_error("Cannot open I2C");
        ioctl(fd, I2C_SLAVE, PCA9685_ADDR);
        init();
    }

    ~PCA9685() { if (fd >= 0) close(fd); }

    void init() {
        writeByte(MODE1, RESTART);
        usleep(1000);
        writeByte(MODE1, AI | ALLCALL);
        usleep(1000);
        writeByte(MODE2, OUTDRV);
        usleep(10000);
        writeByte(PRESCALE, 121);
        usleep(1000);
        writeByte(MODE1, RESTART | AI | ALLCALL);
        usleep(10000);
    }

    void setPWM(uint8_t ch, uint16_t on, uint16_t off) {
        uint8_t reg = LED0_ON_L + 4 * ch;
        uint8_t buf[5] = {
            reg,
            static_cast<uint8_t>(off & 0xFF),
            static_cast<uint8_t>((off >> 8) & 0x0F),
            static_cast<uint8_t>(on & 0xFF),
            static_cast<uint8_t>((on >> 8) & 0x0F)
        };
        write(fd, buf, 5);
    }

    void setServoPulse(uint8_t ch, float us) {
        uint16_t tick = static_cast<uint16_t>(us / 20000.0f * 4096.0f);
        if (tick > 4095) tick = 4095;
        setPWM(ch, 0, tick);
    }

    void setAllOff() {
        for (int i = 0; i < 12; i++) setServoPulse(i, 0);
    }
};

class Leg {
private:
    PCA9685& pca;
    int hip_ch, thigh_ch, calf_ch;
    bool inverted;
    
    float degToPulse(float deg) {
        return 1500.0f + deg * 5.56f;
    }

public:
    Leg(PCA9685& p, int h, int t, int c, bool inv = false)
        : pca(p), hip_ch(h), thigh_ch(t), calf_ch(c), inverted(inv) {}

    void setPosition(float hip, float thigh, float calf) {
        if (inverted) {
            hip = -hip;
            thigh = -thigh;
            calf = -calf;
        }
        pca.setServoPulse(hip_ch, degToPulse(hip));
        pca.setServoPulse(thigh_ch, degToPulse(thigh));
        pca.setServoPulse(calf_ch, degToPulse(calf));
    }
};

void msSleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    std::cout << "========================================\n";
    std::cout << "  QUADRUPED GAIT (C++)\n";
    std::cout << "========================================\n\n";

    PCA9685 pca;

    // Legs: hip, thigh, calf channels
    Leg fl(pca, 0, 1, 2, false);
    Leg fr(pca, 3, 4, 5, true);
    Leg bl(pca, 6, 7, 8, false);
    Leg br(pca, 9, 10, 11, true);

    // Stand
    std::cout << "[STAND] Neutral position...\n";
    fl.setPosition(0, 0, 0);
    fr.setPosition(0, 0, 0);
    bl.setPosition(0, 0, 0);
    br.setPosition(0, 0, 0);
    msSleep(2000);

    // Trot gait
    std::cout << "[TROT] 4 cycles...\n";
    int cycles = 4;
    float step_len = 30.0f;
    float step_h = 20.0f;
    int step_time = 300; // ms per step phase

    for (int c = 0; c < cycles; c++) {
        std::cout << "  Cycle " << (c+1) << "/" << cycles << "\n";

        // Phase 1: FL+BR forward, FR+BL back
        std::cout << "    Phase 1: FL+BR fwd\n";
        for (int i = 0; i <= 10; i++) {
            float t = i / 10.0f;
            float h = step_h * std::sin(t * M_PI);
            fl.setPosition(-step_len * t, h, 0);
            br.setPosition(-step_len * t, h, 0);
            fr.setPosition(step_len * (1-t), 0, 0);
            bl.setPosition(step_len * (1-t), 0, 0);
            msSleep(step_time / 10);
        }

        // Phase 2: FR+BL forward, FL+BR back
        std::cout << "    Phase 2: FR+BL fwd\n";
        for (int i = 0; i <= 10; i++) {
            float t = i / 10.0f;
            float h = step_h * std::sin(t * M_PI);
            fr.setPosition(-step_len * t, h, 0);
            bl.setPosition(-step_len * t, h, 0);
            fl.setPosition(step_len * (1-t), 0, 0);
            br.setPosition(step_len * (1-t), 0, 0);
            msSleep(step_time / 10);
        }
    }

    // Stand
    std::cout << "\n[STAND] Returning to neutral...\n";
    fl.setPosition(0, 0, 0);
    fr.setPosition(0, 0, 0);
    bl.setPosition(0, 0, 0);
    br.setPosition(0, 0, 0);
    msSleep(2000);

    std::cout << "\n========================================\n";
    std::cout << "  COMPLETE\n";
    std::cout << "========================================\n";

    return 0;
}

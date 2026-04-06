#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"
#define MODE1 0x00, MODE2 0x01, PRESCALE 0xFE, LED0_ON_L 0x06
#define RESTART 0x80, SLEEP 0x10, ALLCALL 0x01, OUTDRV 0x04, AI 0x20

class PCA9685 {
    int fd;
    bool writeByte(uint8_t r, uint8_t d) {
        uint8_t b[2] = {r, d};
        return write(fd, b, 2) == 2;
    }
public:
    PCA9685() {
        fd = open(I2C_BUS, O_RDWR);
        ioctl(fd, I2C_SLAVE, PCA9685_ADDR);
        writeByte(0x00, RESTART);
        usleep(1000);
        writeByte(0x00, AI | ALLCALL);
        usleep(1000);
        writeByte(0x01, OUTDRV);
        writeByte(0xFE, 121);
        writeByte(0x00, RESTART | AI | ALLCALL);
        usleep(10000);
    }
    void setPWM(uint8_t ch, uint16_t on, uint16_t off) {
        uint8_t reg = 0x06 + 4 * ch;
        uint8_t buf[5] = {reg, (uint8_t)(off&0xFF), (uint8_t)((off>>8)&0x0F), (uint8_t)(on&0xFF), (uint8_t)((on>>8)&0x0F)};
        write(fd, buf, 5);
    }
    void setPulse(uint8_t ch, float us) {
        uint16_t t = (uint16_t)(us / 20000.0f * 4096.0f);
        if(t > 4095) t = 4095;
        setPWM(ch, 0, t);
    }
};

int main() {
    PCA9685 pca;
    std::cout << "TEST: BIG ANGLES (±90 degrees)\n";
    std::cout << "Watch each leg move aggressively!\n\n";
    
    // FL leg (0,1,2)
    std::cout << "FL leg: 1000us -> 2000us\n";
    for(int i=0; i<3; i++) {
        pca.setPulse(i, 1000);
        pca.setPulse(i, 2000);
    }
    usleep(2000000);
    for(int i=0; i<3; i++) pca.setPulse(i, 1500);
    usleep(1000000);
    
    // FR leg (3,4,5)
    std::cout << "FR leg: 1000us -> 2000us\n";
    for(int i=3; i<6; i++) {
        pca.setPulse(i, 1000);
        pca.setPulse(i, 2000);
    }
    usleep(2000000);
    for(int i=3; i<6; i++) pca.setPulse(i, 1500);
    usleep(1000000);
    
    std::cout << "TEST COMPLETE - all servos at neutral\n";
    return 0;
}

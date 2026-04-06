#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"
#define MODE1 0x00
#define MODE2 0x01
#define PRESCALE 0xFE
#define LED0_ON_L 0x06
#define RESTART 0x80
#define SLEEP 0x10
#define ALLCALL 0x01
#define OUTDRV 0x04
#define AI 0x20

class PCA9685 {
    int fd;
    bool writeByte(uint8_t reg, uint8_t data) {
        uint8_t buf[2] = {reg, data};
        return write(fd, buf, 2) == 2;
    }
public:
    PCA9685() {
        fd = open(I2C_BUS, O_RDWR);
        ioctl(fd, I2C_SLAVE, PCA9685_ADDR);
        writeByte(MODE1, RESTART);
        usleep(1000);
        writeByte(MODE1, AI | ALLCALL);
        usleep(1000);
        writeByte(MODE2, OUTDRV);
        writeByte(PRESCALE, 121);
        writeByte(MODE1, RESTART | AI | ALLCALL);
        usleep(10000);
    }
    void setPWM(uint8_t ch, uint16_t on, uint16_t off) {
        uint8_t reg = LED0_ON_L + 4 * ch;
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
    std::cout << "AGGRESSIVE SERVO TEST\n";
    std::cout << "Each leg will move 1000us -> 2000us\n\n";
    
    for(int leg=0; leg<4; leg++) {
        int base = leg * 3;
        std::cout << "LEG " << leg << ": ";
        
        for(int ch=base; ch<base+3; ch++) {
            pca.setPulse(ch, 1000);
        }
        std::cout << "1000us (wait 2s) ";
        std::cout.flush();
        sleep(2);
        
        for(int ch=base; ch<base+3; ch++) {
            pca.setPulse(ch, 2000);
        }
        std::cout << "2000us (wait 2s) ";
        std::cout.flush();
        sleep(2);
        
        for(int ch=base; ch<base+3; ch++) {
            pca.setPulse(ch, 1500);
        }
        std::cout << "1500us\n";
    }
    
    std::cout << "\nDONE\n";
    return 0;
}

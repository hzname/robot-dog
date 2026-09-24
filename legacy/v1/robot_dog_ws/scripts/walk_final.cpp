#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include <chrono>

#define PCA9685_ADDR 0x40
#define I2C_BUS "/dev/i2c-0"

class PCA9685 {
    int fd;
public:
    PCA9685() {
        fd = open(I2C_BUS, O_RDWR);
        ioctl(fd, I2C_SLAVE, PCA9685_ADDR);
        uint8_t init[] = {0x00, 0x80};  // Restart
        write(fd, init, 2);
        usleep(10000);
        uint8_t prescale[] = {0xFE, 121};
        write(fd, prescale, 2);
        usleep(10000);
        uint8_t mode2[] = {0x01, 0x04};  // OUTDRV
        write(fd, mode2, 2);
        uint8_t mode1[] = {0x00, 0x21};  // AI + ALLCALL
        write(fd, mode1, 2);
        usleep(100000);
    }
    ~PCA9685() { close(fd); }
    
    // CORRECT ORDER: ON_L, ON_H, OFF_L, OFF_H
    void setPWM(uint8_t ch, uint16_t on, uint16_t off) {
        uint8_t reg = 0x06 + 4 * ch;
        uint8_t buf[5] = {
            reg,
            (uint8_t)(on & 0xFF),
            (uint8_t)((on >> 8) & 0x0F),
            (uint8_t)(off & 0xFF),
            (uint8_t)((off >> 8) & 0x0F)
        };
        write(fd, buf, 5);
    }
    
    void setPulse(uint8_t ch, float us) {
        uint16_t t = (uint16_t)(us / 20000.0f * 4096.0f);
        if(t > 4095) t = 4095;
        setPWM(ch, 0, t);
    }
};

class Leg {
    PCA9685& pca;
    int h, t, c;
    bool inv;
    float d2p(float d) { return 1500.0f + d * 5.56f; }
public:
    Leg(PCA9685& p, int hi, int th, int ca, bool i=false): pca(p), h(hi), t(th), c(ca), inv(i) {}
    void set(float hip, float thigh, float calf) {
        if(inv) { hip=-hip; thigh=-thigh; calf=-calf; }
        pca.setPulse(h, d2p(hip));
        pca.setPulse(t, d2p(thigh));
        pca.setPulse(c, d2p(calf));
    }
};

void ms(int m) { std::this_thread::sleep_for(std::chrono::milliseconds(m)); }

int main() {
    std::cout << "QUADRUPED GAIT (FIXED)\n";
    PCA9685 pca;
    
    Leg fl(pca, 0,1,2,false), fr(pca,3,4,5,true);
    Leg bl(pca, 6,7,8,false), br(pca,9,10,11,true);
    
    // Stand
    std::cout << "Standing...\n";
    fl.set(0,0,0); fr.set(0,0,0); bl.set(0,0,0); br.set(0,0,0);
    ms(2000);
    
    // Trot
    std::cout << "Trot gait - 4 cycles\n";
    float len=30.0f, h=20.0f, t=300.0f;
    for(int cyc=0; cyc<4; cyc++) {
        std::cout << "Cycle " << cyc+1 << "\n";
        
        // FL+BR fwd
        std::cout << "  FL+BR forward\n";
        for(int i=0; i<=10; i++) {
            float tt = i/10.0f;
            float hh = h*std::sin(tt*M_PI);
            fl.set(-len*tt, hh, 0);
            br.set(-len*tt, hh, 0);
            fr.set(len*(1-tt), 0, 0);
            bl.set(len*(1-tt), 0, 0);
            ms(t/10);
        }
        
        // FR+BL fwd
        std::cout << "  FR+BL forward\n";
        for(int i=0; i<=10; i++) {
            float tt = i/10.0f;
            float hh = h*std::sin(tt*M_PI);
            fr.set(-len*tt, hh, 0);
            bl.set(-len*tt, hh, 0);
            fl.set(len*(1-tt), 0, 0);
            br.set(len*(1-tt), 0, 0);
            ms(t/10);
        }
    }
    
    // Stand
    std::cout << "Standing...\n";
    fl.set(0,0,0); fr.set(0,0,0); bl.set(0,0,0); br.set(0,0,0);
    ms(2000);
    
    std::cout << "COMPLETE\n";
    return 0;
}

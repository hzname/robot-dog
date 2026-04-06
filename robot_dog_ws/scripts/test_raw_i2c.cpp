#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

int main() {
    const char* bus = "/dev/i2c-0";
    int addr = 0x40;
    
    int fd = open(bus, O_RDWR);
    if (fd < 0) {
        std::cerr << "Cannot open " << bus << std::endl;
        return 1;
    }
    
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        std::cerr << "Cannot set address" << std::endl;
        close(fd);
        return 1;
    }
    
    // Инициализация
    uint8_t init_seq[] = {0x00, 0x81};  // MODE1 = restart + allcall
    write(fd, init_seq, 2);
    usleep(50000);
    
    uint8_t prescale[] = {0xFE, 121};  // 50Hz
    write(fd, prescale, 2);
    usleep(10000);
    
    uint8_t mode2[] = {0x01, 0x04};  // MODE2 = outdrv
    write(fd, mode2, 2);
    
    // Канал 0 - всегда HIGH (100%)
    std::cout << "Setting channel 0 to ALWAYS HIGH (100%)" << std::endl;
    uint8_t data[] = {0x06, 0x00, 0x00, 0x0F, 0xFF};  // LED0_ON_L: ON=0, OFF=4095
    write(fd, data, 5);
    
    std::cout << "Measure channel 0 output for 5 seconds..." << std::endl;
    sleep(5);
    
    // Канал 0 - 50%
    std::cout << "\nSetting channel 0 to 50% PWM" << std::endl;
    uint8_t data50[] = {0x06, 0x00, 0x00, 0x00, 0x08};  // ON=0, OFF=2048
    write(fd, data50, 5);
    
    std::cout << "Measure channel 0 output for 5 seconds..." << std::endl;
    sleep(5);
    
    // Читаем что записали
    uint8_t reg = 0x06;
    write(fd, &reg, 1);
    uint8_t buf[4];
    read(fd, buf, 4);
    
    uint16_t on = buf[1] << 8 | buf[0];
    uint16_t off = buf[3] << 8 | buf[2];
    std::cout << "\nRead back: ON=" << on << ", OFF=" << off << std::endl;
    std::cout << "Expected: ON=0, OFF=2048" << std::endl;
    
    // Выключаем
    uint8_t off_data[] = {0x06, 0x00, 0x00, 0x00, 0x00};
    write(fd, off_data, 5);
    
    close(fd);
    return 0;
}

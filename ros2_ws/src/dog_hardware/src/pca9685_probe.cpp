// pca9685_probe: bring-up helper that talks to the PCA9685 without ROS.
//
//   pca9685_probe [--device /dev/i2c-0] [--address 0x40] <command>
//     check                 open the chip and report its PWM frequency
//     pulse <ch> <us>       output a pulse width on one channel (500..2500 us)
//     sweep <ch> <us1> <us2> slowly sweep one channel between two pulses
//     off [ch]              turn one channel (or all) off
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "dog_hardware/servo_bus.hpp"

namespace
{
int usage()
{
  std::cerr <<
    "usage: pca9685_probe [--device /dev/i2c-0] [--address 0x40] <command>\n"
    "  check                    open the chip and print its PWM frequency\n"
    "  pulse <ch> <us>          output a pulse on one channel (500..2500 us)\n"
    "  sweep <ch> <us1> <us2>   sweep a channel slowly between two pulses\n"
    "  off [ch]                 disable one channel or all channels\n";
  return 2;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string device = "/dev/i2c-0";
  int address = 0x40;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--device" && i + 1 < argc) {device = argv[++i];}
    else if (a == "--address" && i + 1 < argc) {address = std::stoi(argv[++i], nullptr, 0);}
    else {args.push_back(a);}
  }
  if (args.empty()) {return usage();}

  dog_hardware::Pca9685Bus bus;
  std::string error;
  if (!bus.open(device, address, 50.0, 25e6, error)) {
    std::cerr << "error: " << error << "\n";
    return 1;
  }
  std::cout << bus.describe() << "\n";

  const std::string cmd = args[0];
  if (cmd == "check") {
    return 0;
  }
  if (cmd == "pulse" && args.size() == 3) {
    const int ch = std::stoi(args[1]);
    const double us = std::stod(args[2]);
    if (us < 500 || us > 2500) {
      std::cerr << "pulse must be within 500..2500 us\n";
      return 2;
    }
    return bus.setPulseUs(ch, us) ? 0 : 1;
  }
  if (cmd == "sweep" && args.size() == 4) {
    const int ch = std::stoi(args[1]);
    const double a = std::stod(args[2]);
    const double b = std::stod(args[3]);
    if (a < 500 || a > 2500 || b < 500 || b > 2500) {
      std::cerr << "pulses must be within 500..2500 us\n";
      return 2;
    }
    const double step = (b > a ? 5.0 : -5.0);  // ~0.5 deg per 20 ms
    for (double us = a; (step > 0 ? us <= b : us >= b); us += step) {
      bus.setPulseUs(ch, us);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
  }
  if (cmd == "off") {
    return (args.size() == 2 ? bus.disable(std::stoi(args[1])) : bus.disableAll()) ? 0 : 1;
  }
  return usage();
}

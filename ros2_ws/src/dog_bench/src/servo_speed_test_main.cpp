// servo_speed_test: measure the servo speed from INA219 current traces,
// without ROS.
//
//   servo_speed_test <selftest|dry-run|run> --ina-address N --shunt-ohm X
//                    [--device /dev/i2c-N] [--samples N]
//
//     selftest       read the INA219 only and check that the polling rate
//                    reaches 1 kHz; PWM is never enabled in this build
//     dry-run, run   reserved for later plans: not available in this build
//
// Exit codes: 0 ok, 1 runtime error or selftest FAIL, 2 bad arguments,
// 3 emergency servo release (reserved; nothing here can enable a servo).
// selftest rewrites the INA219 configuration, so stop the robot stack first
// (docker compose stop).
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/selftest.hpp"

namespace
{

struct Args
{
  std::string mode;
  std::string device{"/dev/i2c-0"};
  bool have_address{false};
  int address{0};
  bool have_shunt{false};
  double shunt_ohm{0.0};
  int samples{dog_bench::kSelftestDefaultSamples};
  bool help{false};
};

void printUsage(std::ostream & os)
{
  os << "usage: servo_speed_test <selftest|dry-run|run> --ina-address N --shunt-ohm X\n"
    << "                        [--device /dev/i2c-N] [--samples N]\n"
    << "  selftest          INA219 only: check that the polling rate reaches 1 kHz\n"
    << "                    (PWM is never enabled in this build; stop the stack first)\n"
    << "  dry-run, run      reserved: not available in this build, servos stay off\n"
    << "  --ina-address N   INA219 address 0x41..0x4f (mandatory; 0x40 is the PCA9685)\n"
    << "  --shunt-ohm X     shunt 0.1 (R100) or 0.01 (R010) Ohm, mandatory\n"
    << "  --device PATH     /dev/i2c-<digits>, default /dev/i2c-0\n"
    << "  --samples N       selftest ticks, 100..20000, default 2000\n"
    << "  --help            print this text and exit 0\n"
    << "exit codes: 0 ok, 1 runtime error or selftest FAIL, 2 bad arguments,\n"
    << "            3 emergency servo release (reserved for a later plan)\n";
}

int argError(const std::string & message)
{
  std::cerr << "error: " << message << "\n";
  printUsage(std::cerr);
  return 2;
}

/// Strict integer: non-empty, no leading space, the whole string is the number,
/// decimal or 0x-prefixed, inside long range. No exceptions.
bool parseInteger(const std::string & text, long & out)
{
  if (text.empty() || std::isspace(static_cast<unsigned char>(text[0]))) {return false;}
  errno = 0;
  char * end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 0);
  if (end != text.c_str() + text.size() || errno == ERANGE) {return false;}
  out = value;
  return true;
}

/// Strict finite double: non-empty, no leading space, the whole string is the
/// number; nan and inf are rejected. No exceptions.
bool parseDouble(const std::string & text, double & out)
{
  if (text.empty() || std::isspace(static_cast<unsigned char>(text[0]))) {return false;}
  errno = 0;
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || errno == ERANGE) {return false;}
  if (!std::isfinite(value)) {return false;}
  out = value;
  return true;
}

/// Only /dev/i2c-<digits>: an arbitrary file is never opened.
bool validDevicePath(const std::string & path)
{
  const std::string prefix = "/dev/i2c-";
  if (path.size() <= prefix.size() || path.compare(0, prefix.size(), prefix) != 0) {return false;}
  for (std::size_t i = prefix.size(); i < path.size(); ++i) {
    if (std::isdigit(static_cast<unsigned char>(path[i])) == 0) {return false;}
  }
  return true;
}

/// Returns 0 and fills args when the command line is good, otherwise 2.
int parseArguments(int argc, char ** argv, Args & args)
{
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--help") {args.help = true; continue;}
    if (option == "--device" || option == "--ina-address" ||
      option == "--shunt-ohm" || option == "--samples") {
      if (i + 1 >= argc) {return argError("option " + option + " needs a value");}
      const std::string value = argv[++i];  // the next argument, even if it starts with '-'
      if (option == "--device") {
        args.device = value;
      } else if (option == "--ina-address") {
        long number = 0;
        if (!parseInteger(value, number)) {return argError("--ina-address: bad number '" + value + "'");}
        args.address = static_cast<int>(number);
        args.have_address = true;
      } else if (option == "--shunt-ohm") {
        double number = 0.0;
        if (!parseDouble(value, number)) {return argError("--shunt-ohm: bad number '" + value + "'");}
        args.shunt_ohm = number;
        args.have_shunt = true;
      } else {
        long number = 0;
        if (!parseInteger(value, number) ||
          number < dog_bench::kSelftestMinSamples || number > dog_bench::kSelftestMaxSamples) {
          return argError("--samples must be an integer in 100..20000");
        }
        args.samples = static_cast<int>(number);
      }
    } else if (!option.empty() && option[0] == '-') {
      return argError("unknown option " + option);
    } else {
      positional.push_back(option);
    }
  }
  if (args.help) {return 0;}  // --help wins over everything else
  if (positional.empty()) {return argError("missing mode (selftest, dry-run, run)");}
  if (positional.size() > 1) {return argError("exactly one mode argument is allowed");}
  args.mode = positional[0];
  return 0;
}

/// Every argument check happens before the bus is opened, for all modes.
int validate(const Args & args, uint16_t & config)
{
  if (args.mode != "selftest" && args.mode != "dry-run" && args.mode != "run") {
    return argError("unknown mode '" + args.mode + "' (selftest, dry-run, run)");
  }
  if (!args.have_address) {return argError("--ina-address is mandatory (0x41..0x4f)");}
  if (!args.have_shunt) {return argError("--shunt-ohm is mandatory (0.1 or 0.01 Ohm)");}
  if (!dog_bench::ina::isSafeAddress(args.address)) {
    if (args.address == dog_bench::ina::kPca9685Address) {
      return argError("--ina-address 0x40 is the PCA9685 and is never probed as an INA;"
        " use 0x41..0x4f");
    }
    return argError("--ina-address must be 0x41..0x4f");
  }
  if (!dog_bench::ina::configForShunt(args.shunt_ohm, config)) {
    return argError("--shunt-ohm must be 0.1 (R100) or 0.01 (R010) Ohm with the matching PGA;"
      " other shunts are not guessed");
  }
  if (!validDevicePath(args.device)) {
    return argError("--device must look like /dev/i2c-<digits>");
  }
  return 0;
}

int selftestMode(const Args & args, uint16_t config)
{
  dog_bench::LinuxI2cBus bus(args.device);
  if (!bus.ok()) {
    std::cerr << "error: cannot open " << args.device << ": " << bus.error() << "\n";
    return 1;
  }
  dog_bench::Ina219Fast ina(bus, args.address);
  if (!ina.configure(config)) {
    std::cerr << "error: " << ina.error() << "\n";
    return 1;
  }
  std::cout << "INA219 at 0x" << std::hex << args.address << std::dec
    << " on " << args.device
    << ", shunt " << args.shunt_ohm * 1000.0 << " mOhm"
    << ", config 0x" << std::hex << config << std::dec
    << "; PWM is not enabled, stop the robot stack first (docker compose stop)\n";

  const dog_bench::SelftestResult r = dog_bench::runSelftest(
    ina, args.samples, dog_bench::monotonicSeconds, dog_bench::sleepSeconds);
  const double hz = r.median_ms > 0.0 ? 1000.0 / r.median_ms : 0.0;
  const double ma_min = dog_bench::Ina219Fast::currentA(
    static_cast<uint16_t>(r.shunt_raw_min), args.shunt_ohm) * 1000.0;
  const double ma_max = dog_bench::Ina219Fast::currentA(
    static_cast<uint16_t>(r.shunt_raw_max), args.shunt_ohm) * 1000.0;
  std::cout << "ticks " << r.samples << "  median " << r.median_ms << " ms (" << hz << " Hz)"
    << "  p99 " << r.p99_ms << " ms  max " << r.max_ms << " ms  errors " << r.errors
    << "  bus " << r.bus_volts << " V"
    << "  shunt " << r.shunt_raw_min << ".." << r.shunt_raw_max << " raw ("
    << ma_min << ".." << ma_max << " mA)\n";
  if (r.ok) {
    std::cout << "PASS selftest\n";
    return 0;
  }
  std::cerr << "FAIL selftest: " << r.reason << "\n";
  return 1;
}

int run(int argc, char ** argv)
{
  Args args;
  if (int code = parseArguments(argc, argv, args); code != 0) {return code;}
  if (args.help) {
    printUsage(std::cout);
    return 0;
  }
  uint16_t config = 0;
  if (int code = validate(args, config); code != 0) {return code;}
  if (args.mode != "selftest") {
    std::cerr << "error: mode '" << args.mode
      << "' is not available in this build: no PWM code is linked, the servos stay off\n";
    return 2;
  }
  return selftestMode(args, config);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 1;
  }
}

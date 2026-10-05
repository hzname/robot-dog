// servo_speed_test: measure the servo speed from INA219 current traces,
// without ROS.
//
//   servo_speed_test <selftest|dry-run|run> --ina-address N --shunt-ohm X
//                    [--device /dev/i2c-N] [--samples N]
//   servo_speed_test <dry-run|run> --ina-address N --shunt-ohm X --channel N
//                    [--pca-address 0x40] [--amp-deg D] [--center-us US]
//                    [--us-per-deg U] [--speeds V,V,..] [--max-seconds S]
//                    [--out PATH] [--yes]
//
//     selftest       read the INA219 only and check that the polling rate
//                    reaches 1 kHz; PWM is never enabled
//     dry-run        print the measurement plan and touch nothing
//     run            drive the ramp on one PCA9685 channel and record the CSV
//
// Exit codes: 0 ok, 1 runtime error or a refused run, 2 bad arguments,
// 3 emergency servo release (the outputs could not be taken down).
// run needs the operator's confirmation (type YES or pass --yes) and a
// known shunt (0.1 or 0.01 Ohm); stop the robot stack first
// (docker compose stop).
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <unistd.h>
#include <vector>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/pwm_out.hpp"
#include "dog_bench/ramp.hpp"
#include "dog_bench/selftest.hpp"
#include "dog_bench/session.hpp"

namespace
{

using dog_bench::kConfirmPrompt;
using dog_bench::kMinPollRateHz;
using dog_bench::Session;
using dog_bench::SessionConfig;
using dog_bench::SessionEnv;
using dog_bench::SessionResult;

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

  // The measurement flags below are for dry-run and run only; selftest
  // refuses them (exit 2).
  bool have_measurement_flags{false};
  int pca_address{dog_bench::pca::kAddress};
  bool have_channel{false};
  int channel{-1};
  double amp_deg{25.0};
  double center_us{1370.0};
  double us_per_deg{9.444};
  bool have_speeds{false};
  std::vector<double> speeds;
  double max_seconds{600.0};
  bool have_out{false};
  std::string out_path;
  bool yes{false};
};

/// Owns the emergency release descriptor; declared before the SignalGuard in
/// main, so it is closed after the handlers are restored (D-10).
class EmergencyFd
{
public:
  EmergencyFd() = default;
  ~EmergencyFd()
  {
    if (fd_ >= 0) {::close(fd_);}
  }
  EmergencyFd(const EmergencyFd &) = delete;
  EmergencyFd & operator=(const EmergencyFd &) = delete;

  void reset(int fd) {fd_ = fd;}
  int get() const {return fd_;}

private:
  int fd_{-1};
};

std::string number(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", value);
  return buf;
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

/// Comma-separated numbers without spaces: "3,3.5,4".
bool parseSpeeds(const std::string & text, std::vector<double> & out)
{
  out.clear();
  if (text.empty()) {return false;}
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = text.find(',', start);
    const std::string part = comma == std::string::npos ?
      text.substr(start) : text.substr(start, comma - start);
    double value = 0.0;
    if (!parseDouble(part, value)) {return false;}
    out.push_back(value);
    if (comma == std::string::npos) {break;}
    start = comma + 1;
  }
  return !out.empty();
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

/// True when the path already exists (a file or a directory).
bool pathExists(const std::string & path)
{
  return ::access(path.c_str(), F_OK) == 0;
}

void printUsage(std::ostream & os)
{
  os << "usage: servo_speed_test <selftest|dry-run|run> --ina-address N --shunt-ohm X\n"
    << "                         [--device /dev/i2c-N] [--samples N]\n"
    << "       servo_speed_test <dry-run|run> --ina-address N --shunt-ohm X --channel N\n"
    << "                         [--pca-address 0x40] [--amp-deg D] [--center-us US]\n"
    << "                         [--us-per-deg U] [--speeds V,V,..] [--max-seconds S]\n"
    << "                         [--out PATH] [--yes]\n"
    << "  selftest          INA219 only: check that the polling rate reaches 1 kHz\n"
    << "                    (stop the robot stack first: docker compose stop)\n"
    << "  dry-run           print the measurement plan; nothing is written to the bus\n"
    << "  run               drive the ramp on one PCA9685 channel and record the CSV\n"
    << "  --ina-address N   INA219 address 0x41..0x4f (mandatory; 0x40 is the PCA9685)\n"
    << "  --shunt-ohm X     shunt 0.1 (R100) or 0.01 (R010) Ohm, mandatory\n"
    << "  --device PATH     /dev/i2c-<digits>, default /dev/i2c-0\n"
    << "  --samples N       selftest ticks, 100..20000, default 2000\n"
    << "  --pca-address N   only 0x40, the chip this tool drives\n"
    << "  --channel N       one servo channel 0..15 (mandatory for dry-run and run)\n"
    << "  --amp-deg D       sweep half-amplitude [deg], at most 30 (default 25)\n"
    << "  --center-us US    pulse at the centre [us], 1000..1800 (default 1370)\n"
    << "  --us-per-deg U    protractor calibration [us/deg], 5..15 (default 9.444)\n"
    << "  --speeds V,V,..   speed grid [rad/s], ascending, default 1.5..10\n"
    << "  --max-seconds S   global timeout [s]; the planned time plus 5 s must fit\n"
    << "  --out PATH        CSV path for run (mandatory); PATH.meta.json gets the meta\n"
    << "  --yes             skip the confirmation (the servo will move)\n"
    << "  --help            print this text and exit 0\n"
    << "exit codes: 0 ok, 1 runtime error or a refused run, 2 bad arguments,\n"
    << "            3 emergency servo release (the outputs could not be taken down)\n";
}

int argError(const std::string & message)
{
  std::cerr << "error: " << message << "\n";
  printUsage(std::cerr);
  return 2;
}

/// Returns 0 and fills args when the command line is good, otherwise 2.
int parseArguments(int argc, char ** argv, Args & args)
{
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--help") {args.help = true; continue;}
    if (option == "--yes") {
      args.yes = true;
      args.have_measurement_flags = true;
      continue;
    }
    if (option == "--device" || option == "--ina-address" || option == "--shunt-ohm" ||
      option == "--samples" || option == "--pca-address" || option == "--channel" ||
      option == "--amp-deg" || option == "--center-us" || option == "--us-per-deg" ||
      option == "--speeds" || option == "--max-seconds" || option == "--out") {
      if (i + 1 >= argc) {return argError("option " + option + " needs a value");}
      const std::string value = argv[++i];  // the next argument, even if it starts with '-'
      if (option == "--device") {
        args.device = value;
      } else if (option == "--ina-address") {
        long parsed = 0;
        if (!parseInteger(value, parsed)) {return argError("--ina-address: bad number '" + value + "'");}
        args.address = static_cast<int>(parsed);
        args.have_address = true;
      } else if (option == "--shunt-ohm") {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {return argError("--shunt-ohm: bad number '" + value + "'");}
        args.shunt_ohm = parsed;
        args.have_shunt = true;
      } else if (option == "--samples") {
        long parsed = 0;
        if (!parseInteger(value, parsed) ||
          parsed < dog_bench::kSelftestMinSamples || parsed > dog_bench::kSelftestMaxSamples) {
          return argError("--samples must be an integer in 100..20000");
        }
        args.samples = static_cast<int>(parsed);
      } else if (option == "--pca-address") {
        long parsed = 0;
        if (!parseInteger(value, parsed)) {return argError("--pca-address: bad number '" + value + "'");}
        args.pca_address = static_cast<int>(parsed);
        args.have_measurement_flags = true;
      } else if (option == "--channel") {
        if (args.have_channel) {
          return argError("--channel was given more than once (exactly one channel)");
        }
        long parsed = 0;
        if (!parseInteger(value, parsed)) {return argError("--channel: bad number '" + value + "'");}
        args.channel = static_cast<int>(parsed);
        args.have_channel = true;
        args.have_measurement_flags = true;
      } else if (option == "--amp-deg") {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {return argError("--amp-deg: bad number '" + value + "'");}
        args.amp_deg = parsed;
        args.have_measurement_flags = true;
      } else if (option == "--center-us") {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {return argError("--center-us: bad number '" + value + "'");}
        args.center_us = parsed;
        args.have_measurement_flags = true;
      } else if (option == "--us-per-deg") {
        double parsed = 0.0;
        if (!parseDouble(value, parsed)) {return argError("--us-per-deg: bad number '" + value + "'");}
        args.us_per_deg = parsed;
        args.have_measurement_flags = true;
      } else if (option == "--speeds") {
        if (!parseSpeeds(value, args.speeds)) {
          return argError("--speeds: bad list '" + value + "' (comma-separated, no spaces)");
        }
        args.have_speeds = true;
        args.have_measurement_flags = true;
      } else if (option == "--max-seconds") {
        long parsed = 0;
        if (!parseInteger(value, parsed)) {return argError("--max-seconds: bad number '" + value + "'");}
        args.max_seconds = static_cast<double>(parsed);
        args.have_measurement_flags = true;
      } else {  // --out
        args.out_path = value;
        args.have_out = true;
        args.have_measurement_flags = true;
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

/// The measurement configuration the flags describe.
SessionConfig measurementConfig(const Args & args, uint16_t config)
{
  SessionConfig cfg;
  cfg.ramp.amp_deg = args.amp_deg;
  cfg.ramp.center_us = args.center_us;
  cfg.ramp.us_per_deg = args.us_per_deg;
  cfg.speeds = args.have_speeds ? args.speeds : dog_bench::Ramp::defaultSpeeds();
  cfg.safety.max_seconds = args.max_seconds;
  cfg.shunt_ohm = args.shunt_ohm;
  cfg.ina_config = config;
  cfg.channel = args.channel;
  cfg.confirmed = false;
  cfg.out_path = args.out_path;
  return cfg;
}

/// Every argument check happens before the bus is opened and before stdin is
/// read, for all modes. Returns 0 or 2.
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
  if (args.mode == "selftest") {
    if (args.have_measurement_flags) {
      return argError("--channel, --out and the measurement flags are only for dry-run and run");
    }
    return 0;
  }

  // dry-run and run share the measurement checks.
  if (args.pca_address != dog_bench::pca::kAddress) {
    return argError("--pca-address must be 0x40, the PCA9685 this tool drives");
  }
  if (!args.have_channel) {
    return argError("--channel is mandatory for dry-run and run (exactly one channel 0..15)");
  }
  if (args.mode == "run" && !args.have_out) {
    return argError("--out is mandatory for run (the CSV path)");
  }
  const SessionConfig cfg = measurementConfig(args, config);
  const std::string config_error = cfg.validate();
  if (!config_error.empty()) {return argError(config_error);}
  const dog_bench::Ramp ramp(cfg.ramp, cfg.speeds);
  const double planned_s = ramp.plannedDurationS();
  if (!(args.max_seconds >= planned_s + 5.0)) {
    return argError("--max-seconds " + number(args.max_seconds) + " s is below the planned " +
      number(planned_s) + " s plus 5 s");
  }
  if (args.mode == "run" && !args.out_path.empty() && pathExists(args.out_path)) {
    return argError("--out '" + args.out_path + "' already exists; pick a fresh path");
  }
  return 0;
}

/// The measurement plan printed by dry-run and run.
void printPlan(const Args & args, const SessionConfig & cfg, double planned_s)
{
  const double saturation_a = cfg.safety.hard_fraction *
    dog_bench::ina::fullScaleShuntVolts(cfg.ina_config) / cfg.shunt_ohm;
  std::cout << "INA219: address 0x" << std::hex << args.address << std::dec
    << ", shunt " << cfg.shunt_ohm * 1000.0 << " mOhm"
    << ", config 0x" << std::hex << cfg.ina_config << std::dec
    << ", saturation at " << cfg.safety.hard_fraction * 100.0 << "% of the scale: "
    << saturation_a << " A\n";
  std::cout << "PWM: PCA9685 at 0x" << std::hex << dog_bench::pca::kAddress << std::dec
    << ", channel " << cfg.channel
    << ", pulses " << cfg.ramp.center_us - cfg.ramp.amp_deg * cfg.ramp.us_per_deg
    << ".." << cfg.ramp.center_us + cfg.ramp.amp_deg * cfg.ramp.us_per_deg << " us"
    << ", centre " << cfg.ramp.center_us << " us, amplitude " << cfg.ramp.amp_deg
    << " deg (" << cfg.ramp.us_per_deg << " us/deg)\n";
  std::cout << "speeds:";
  for (const double v : cfg.speeds) {std::cout << " " << v;}
  std::cout << " rad/s\n";
  std::cout << "time: planned " << planned_s << " s, max " << cfg.safety.max_seconds << " s\n";
  std::cout << "safety: overcurrent " << cfg.safety.overcurrent_a << " A for "
    << cfg.safety.overcurrent_time_s << " s; shunt plausibility "
    << cfg.safety.plausibility_min_a << ".." << cfg.safety.plausibility_max_a << " A; "
    << cfg.safety.max_consecutive_ina_errors << " consecutive INA errors; PCA write error;"
    << " tick overrun " << cfg.safety.max_tick_overrun_s << " s\n";
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

}  // namespace

int main(int argc, char ** argv)
{
  try {
    Args args;
    if (int code = parseArguments(argc, argv, args); code != 0) {return code;}
    if (args.help) {
      printUsage(std::cout);
      return 0;
    }
    uint16_t config = 0;
    if (int code = validate(args, config); code != 0) {return code;}
    if (args.mode == "selftest") {return selftestMode(args, config);}

    // dry-run and run share the plan; dry-run touches nothing at all.
    SessionConfig cfg = measurementConfig(args, config);
    const dog_bench::Ramp ramp(cfg.ramp, cfg.speeds);
    printPlan(args, cfg, ramp.plannedDurationS());
    if (args.mode == "dry-run") {
      std::cout << "dry-run: nothing was written to the bus\n";
      return 0;
    }

    // ---- run: the confirmation first, then the bus in the plan's order -----
    if (!args.yes) {
      std::cout << kConfirmPrompt << "\n" << std::flush;
      std::string line;
      if (!std::getline(std::cin, line) || !dog_bench::isConfirmed(line)) {
        std::cerr << "not confirmed: nothing was written to the bus\n";
        return 1;
      }
    }
    cfg.confirmed = true;

    // Declarations in destruction order: the emergency descriptor is closed
    // last, the SignalGuard before it, the PWM output before the bus (D-10).
    EmergencyFd emergency;
    std::optional<dog_bench::SignalGuard> guard;
    std::optional<dog_bench::LinuxI2cBus> bus;
    std::optional<dog_bench::Ina219Fast> ina;
    std::optional<dog_bench::Pca9685Out> pwm;

    bus.emplace(args.device);
    if (!bus->ok()) {
      std::cerr << "error: cannot open " << args.device << ": " << bus->error() << "\n";
      return 1;
    }
    ina.emplace(*bus, args.address);
    if (!ina->configure(config)) {
      std::cerr << "error: " << ina->error() << "\n";
      return 1;
    }
    pwm.emplace(*bus, args.channel, ramp.lowUs(), ramp.highUs());
    std::string emergency_error;
    const int emergency_fd = dog_bench::openEmergencyFd(args.device, emergency_error);
    if (emergency_fd < 0) {
      std::cerr << "error: cannot prepare the emergency release path: " << emergency_error << "\n";
      return 1;
    }
    emergency.reset(emergency_fd);
    guard.emplace();

    Session session(cfg);
    const SessionEnv env = SessionEnv::realtime(*guard, emergency.get());
    const SessionResult result = session.run(*ina, *pwm, env);

    std::cout << result.stop_reason << ": " << result.samples << " rows, "
      << result.duration_s << " s, " << result.rate_hz << " Hz, peak "
      << result.peak_current_a << " A -> " << cfg.out_path << " and "
      << cfg.out_path << ".meta.json\n";
    if (!result.error.empty()) {std::cerr << "error: " << result.error << "\n";}
    if (result.samples > 0 && result.rate_hz < kMinPollRateHz) {
      std::cerr << "WARN: the poll rate " << result.rate_hz << " Hz is below " << kMinPollRateHz
        << " Hz; run selftest and consider the 400 kHz overlay (docs/REVIEW.md item 12)\n";
    }
    if (result.exit_code == 3) {
      std::cerr << "EMERGENCY: the servo may still be driven - cut the servo power now,"
        " then run pca9685_probe off\n";
    }
    return result.exit_code;
  } catch (const std::exception & e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 1;
  }
}

// Measurement session of the servo bench tool: drives the 01-09 speed ramp
// through one PCA9685 channel while polling the INA219 at 1 kHz, records the
// CSV and <csv>.meta.json of the tools/servo_speed contract, and releases the
// outputs on every exit (D-08, D-09, D-10). run() order: validate and confirm
// without touching the bus, preflight, arm the emergency release, a read-only
// pre-roll, the measurement loop, release, files. ROS-free; validation
// returns a message instead of throwing.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/pwm_out.hpp"
#include "dog_bench/ramp.hpp"
#include "dog_bench/safety.hpp"
#include "dog_bench/selftest.hpp"

namespace dog_bench
{

/// Tick of the measurement loop [s]: 1 ms (D-08).
constexpr double kSessionTickS = 0.001;
/// The pulse is rewritten once per this period [s] (the 50 Hz PWM frame).
constexpr double kSessionPwmPeriodS = 0.02;
/// The bus register is also read on every kSessionBusEvery-th tick.
constexpr int kSessionBusEvery = kSelftestBusEvery;
/// Read-only pre-roll ticks before the first pulse is possible (D-10).
constexpr int kSessionPreRollTicks = 50;
/// A measured poll rate below this gets a WARN (D-08).
constexpr double kMinPollRateHz = 500.0;

/// Header of the measurement CSV; the format of tools/servo_speed/analyze.py.
constexpr const char * kCsvHeader = "t_s,stroke_id,direction,cmd_us,shunt_raw,bus_raw";

/// The confirmation question before anything can drive a servo (D-10).
constexpr const char * kConfirmPrompt =
  "питание проверено мультиметром, нога свободна, рука на выключателе? (YES)";

struct SessionConfig
{
  RampParams ramp;
  std::vector<double> speeds{Ramp::defaultSpeeds()};
  SafetyParams safety;
  double shunt_ohm{0.0};   // mandatory, never guessed (D-10)
  uint16_t ina_config{0};  // the INA219 configuration register value
  int channel{-1};         // exactly one channel, 0..15
  bool confirmed{false};   // the operator answered YES (or passed --yes)
  std::string out_path;    // CSV path; <out_path>.meta.json gets the metadata

  /// Returns an empty string when OK, otherwise the first message naming the
  /// offending field (ramp, speeds, safety, channel).
  std::string validate() const;
};

class SignalGuard;

struct SessionEnv
{
  std::function<double()> now;           // monotonic seconds
  std::function<void(double)> pause;     // sleep for the given seconds
  std::function<bool()> stop_requested;  // a soft signal has arrived
  std::function<void()> on_armed;        // arm the emergency release path
  std::function<void()> on_released;     // disarm it after a good release

  /// The real-time environment: monotonicSeconds/sleepSeconds plus the guard
  /// and its emergency descriptor.
  static SessionEnv realtime(SignalGuard & guard, int emergency_fd);
};

struct SessionResult
{
  std::string stop_reason{"invalid_config"};
  int samples{0};          // CSV rows recorded
  int exit_code{2};
  bool released{true};
  double duration_s{0.0};  // from the pre-roll start to the last tick
  double rate_hz{0.0};     // samples / duration_s
  double max_interval_ms{0.0};
  double max_read_ms{0.0};
  double peak_current_a{0.0};
  std::string error;
};

class Session
{
public:
  explicit Session(const SessionConfig & config) : config_(config) {}

  /// Run the measurement (see the header comment for the order). `ina` must
  /// be configured; `pwm` is preflighted inside. Never throws.
  SessionResult run(Ina219Fast & ina, PwmOut & pwm, const SessionEnv & env);

  /// The <csv>.meta.json content of a finished run.
  std::string metaJson(const SessionResult & result) const;

private:
  SessionConfig config_;
};

/// True only for exactly "YES" after trailing \r and \n are stripped (D-10).
bool isConfirmed(const std::string & line);

/// Signal handling of the measurement session (D-10). SIGINT, SIGTERM,
/// SIGHUP and SIGQUIT set a flag the loop reads every tick; SIGSEGV, SIGABRT,
/// SIGBUS and SIGFPE run the fatal handler: one write() of the ALL_LED_OFF
/// frame into the armed emergency descriptor, then _exit(3). The destructor
/// restores every previous handler. Only one instance may exist (the handler
/// state is per-process).
class SignalGuard
{
public:
  SignalGuard();
  ~SignalGuard();
  SignalGuard(const SignalGuard &) = delete;
  SignalGuard & operator=(const SignalGuard &) = delete;

  /// True when every handler and the alternate stack were installed.
  bool ok() const {return ok_;}
  /// True once one of the soft signals has arrived.
  bool stopRequested() const;
  /// The last soft signal number, or 0.
  int lastSignal() const;
  /// Arm the fatal handler with the emergency descriptor (fd >= 0).
  void armEmergency(int fd);
  /// Disarm it again after the outputs were released.
  void disarmEmergency();

private:
  bool ok_{false};
};

/// Open the emergency release descriptor: the device with the PCA9685 slave
/// address selected, so the fatal handler can take every output down with one
/// write(). Returns -1 and fills `error` on failure.
int openEmergencyFd(const std::string & device, std::string & error);

/// The fatal-signal handler: one write() of pca::kAllLedOffBytes into the
/// armed descriptor, then _exit(3). The body only reads statics, writes and
/// exits — nothing else is async-signal-safe.
void onFatalSignal(int signum);

}  // namespace dog_bench

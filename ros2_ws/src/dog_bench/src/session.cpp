#include "dog_bench/session.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

namespace dog_bench
{

namespace
{

// ---- process-wide signal state (one SignalGuard at a time, D-10) ----------

/// The soft signal number; 0 when none has arrived.
volatile std::sig_atomic_t g_stop_signal = 0;
/// The emergency descriptor the fatal handler writes to; -1 when disarmed.
volatile std::sig_atomic_t g_emergency_fd = -1;
/// Alternate stack for the fatal handlers (a stack-overflow SIGSEGV must
/// still have a way out).
uint8_t g_alt_stack[64 * 1024];
bool g_alt_stack_set = false;
stack_t g_old_alt_stack{};
/// The handlers in place before the guard was installed, in the order
/// soft[0..3] then fatal[0..3].
struct sigaction g_old_actions[8];

constexpr int kSoftSignals[4] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT};
constexpr int kFatalSignals[4] = {SIGSEGV, SIGABRT, SIGBUS, SIGFPE};

/// Soft signal: only an async-signal-safe store.
void onSoftSignal(int signum)
{
  g_stop_signal = signum;
}

// ---- small helpers --------------------------------------------------------

std::string number(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", value);
  return buf;
}

/// One recorded CSV row.
struct SessionRow
{
  double t_s;
  int stroke_id;
  int direction;
  double cmd_us;
  int shunt_raw;
  bool has_bus;
  int bus_raw;
};

/// The CSV text of a run; the format of tools/servo_speed/analyze.py.
std::string csvText(const std::vector<SessionRow> & rows)
{
  std::string text(kCsvHeader);
  text += "\n";
  char buf[128];
  for (const SessionRow & row : rows) {
    std::snprintf(buf, sizeof(buf), "%.6f,%d,%d,%.1f,%d,",
      row.t_s, row.stroke_id, row.direction, row.cmd_us, row.shunt_raw);
    text += buf;
    if (row.has_bus) {
      std::snprintf(buf, sizeof(buf), "%d", row.bus_raw);
      text += buf;
    }
    text += "\n";
  }
  return text;
}

/// Write `text` to `path`; false with the reason in `error`.
bool writeTextFile(const std::string & path, const std::string & text, std::string & error)
{
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) {
    error = "cannot open " + path;
    return false;
  }
  out << text;
  if (!out) {
    error = "cannot write " + path;
    return false;
  }
  return true;
}

}  // namespace

// ---- SignalGuard ----------------------------------------------------------

SignalGuard::SignalGuard()
{
  stack_t ss;
  std::memset(&ss, 0, sizeof(ss));
  ss.ss_sp = g_alt_stack;
  ss.ss_size = sizeof(g_alt_stack);
  g_alt_stack_set = sigaltstack(&ss, &g_old_alt_stack) == 0;

  bool installed = g_alt_stack_set;
  for (int i = 0; i < 4; ++i) {
    struct sigaction action;
    std::memset(&action, 0, sizeof(action));
    action.sa_handler = onSoftSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;
    if (sigaction(kSoftSignals[i], &action, &g_old_actions[i]) != 0) {installed = false;}
  }
  for (int i = 0; i < 4; ++i) {
    struct sigaction action;
    std::memset(&action, 0, sizeof(action));
    action.sa_handler = onFatalSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_ONSTACK | SA_RESETHAND;
    if (sigaction(kFatalSignals[i], &action, &g_old_actions[4 + i]) != 0) {installed = false;}
  }
  ok_ = installed;
}

SignalGuard::~SignalGuard()
{
  for (int i = 0; i < 4; ++i) {
    sigaction(kSoftSignals[i], &g_old_actions[i], nullptr);
    sigaction(kFatalSignals[i], &g_old_actions[4 + i], nullptr);
  }
  if (g_alt_stack_set) {
    sigaltstack(&g_old_alt_stack, nullptr);
    g_alt_stack_set = false;
  }
  g_stop_signal = 0;
  g_emergency_fd = -1;
}

bool SignalGuard::stopRequested() const {return g_stop_signal != 0;}

int SignalGuard::lastSignal() const {return static_cast<int>(g_stop_signal);}

void SignalGuard::armEmergency(int fd)
{
  if (fd >= 0) {g_emergency_fd = fd;}
}

void SignalGuard::disarmEmergency() {g_emergency_fd = -1;}

int openEmergencyFd(const std::string & device, std::string & error)
{
  const int fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    error = "cannot open " + device + ": " + std::strerror(errno);
    return -1;
  }
  // A plain write() from the handler needs the slave address selected on the
  // descriptor; the measurement bus keeps the address inside every I2C_RDWR
  // message instead, so this is the only place that selects it.
  if (::ioctl(fd, I2C_SLAVE, pca::kAddress) < 0) {
    error = "cannot select the PCA9685 address on " + device + ": " + std::strerror(errno);
    ::close(fd);
    return -1;
  }
  error.clear();
  return fd;
}

void onFatalSignal(int signum)
{
  (void)signum;
  const int fd = g_emergency_fd;
  if (fd >= 0) {
    const ssize_t written = ::write(fd, pca::kAllLedOffBytes.data(), pca::kAllLedOffBytes.size());
    (void)written;
  }
  ::_exit(3);
}

bool isConfirmed(const std::string & line)
{
  std::size_t end = line.size();
  while (end > 0 && (line[end - 1] == '\n' || line[end - 1] == '\r')) {--end;}
  return end == 3 && line.compare(0, 3, "YES") == 0;
}

std::string SessionConfig::validate() const
{
  const std::string ramp_error = ramp.validate();
  if (!ramp_error.empty()) {return ramp_error;}
  const std::string speeds_error = validateSpeeds(speeds);
  if (!speeds_error.empty()) {return speeds_error;}
  const std::string safety_error = validateSafety(safety, shunt_ohm, ina_config);
  if (!safety_error.empty()) {return safety_error;}
  if (channel < 0 || channel >= pca::kPwmChannels) {
    return "channel must be within 0..15 (got " + std::to_string(channel) + ")";
  }
  return "";
}

SessionEnv SessionEnv::realtime(SignalGuard & guard, int emergency_fd)
{
  SessionEnv env;
  env.now = monotonicSeconds;
  env.pause = sleepSeconds;
  env.stop_requested = [&guard]() {return guard.stopRequested();};
  env.on_armed = [&guard, emergency_fd]() {guard.armEmergency(emergency_fd);};
  env.on_released = [&guard]() {guard.disarmEmergency();};
  return env;
}

std::string Session::metaJson(const SessionResult & result) const
{
  std::string out = "{";
  out += "\"shunt_ohm\": " + number(config_.shunt_ohm);
  out += ", \"us_per_deg\": " + number(config_.ramp.us_per_deg);
  out += ", \"amp_deg\": " + number(config_.ramp.amp_deg);
  out += ", \"center_us\": " + number(config_.ramp.center_us);
  out += ", \"channel\": " + std::to_string(config_.channel);
  out += ", \"speeds_rad_s\": [";
  for (std::size_t i = 0; i < config_.speeds.size(); ++i) {
    if (i > 0) {out += ", ";}
    out += number(config_.speeds[i]);
  }
  out += "]";
  out += ", \"strokes_per_speed\": " + std::to_string(kStrokesPerSpeed);
  out += ", \"hold_s\": " + number(config_.ramp.hold_s);
  out += ", \"rest_s\": " + number(config_.ramp.rest_s);
  out += ", \"ina_config\": " + std::to_string(config_.ina_config);
  out += ", \"stop_reason\": \"" + result.stop_reason + "\"}";
  return out;
}

SessionResult Session::run(Ina219Fast & ina, PwmOut & pwm, const SessionEnv & env)
{
  SessionResult result;
  std::string stop_reason;

  // (1) Configuration and confirmation first: no bus, no PWM, no files.
  const std::string config_error = config_.validate();
  if (!config_error.empty()) {
    result.stop_reason = "invalid_config";
    result.error = config_error;
    result.exit_code = 2;
    return result;
  }
  if (!env.now || !env.pause) {
    result.stop_reason = "invalid_config";
    result.error = "the session clock is not set";
    result.exit_code = 2;
    return result;
  }
  if (!config_.confirmed) {
    result.stop_reason = "refused_not_confirmed";
    result.exit_code = 1;
    return result;
  }

  // (2) Preflight; any refusal is refused_foreign_channel, with no pulse.
  if (!pwm.preflight()) {
    result.stop_reason = "refused_foreign_channel";
    result.error = pwm.error();
    result.exit_code = 1;
    return result;
  }

  // (3) Everything up to the release runs inside one try; the release itself
  // stays outside, so an exception can never skip it (D-10).
  SafetyGuard guard(config_.safety, config_.shunt_ohm, config_.ina_config);
  Ramp ramp(config_.ramp, config_.speeds);
  std::vector<SessionRow> rows;
  rows.reserve(static_cast<std::size_t>(ramp.plannedDurationS() / kSessionTickS) +
    static_cast<std::size_t>(kSessionPreRollTicks) + 1024u);

  double t_ref = 0.0;
  double last_t0 = 0.0;
  double previous_t0 = 0.0;
  bool have_tick = false;
  bool loop_started = false;
  double last_cmd_us = ramp.state().cmd_us;
  bool pca_ok = true;
  double peak_current_a = 0.0;
  double max_interval_ms = 0.0;
  double max_read_ms = 0.0;
  int samples = 0;
  const int pwm_every = static_cast<int>(std::lround(kSessionPwmPeriodS / kSessionTickS));

  try {
    if (env.on_armed) {env.on_armed();}
    t_ref = env.now();

    // Read-only pre-roll: every tick feeds the guard, nothing is written and
    // no row is recorded, so an event stops the run before the first pulse.
    for (int i = 0; i < kSessionPreRollTicks && stop_reason.empty(); ++i) {
      const double t0 = env.now();
      if (env.stop_requested && env.stop_requested()) {
        stop_reason = "signal";
        break;
      }
      if (have_tick) {max_interval_ms = std::max(max_interval_ms, (t0 - last_t0) * 1000.0);}
      last_t0 = t0;
      have_tick = true;

      uint16_t raw = 0;
      const double t_before = env.now();
      const bool shunt_ok = ina.readShunt(raw);
      const double t_after = env.now();
      max_read_ms = std::max(max_read_ms, (t_after - t_before) * 1000.0);
      bool bus_ok = true;
      if (i % kSessionBusEvery == kSessionBusEvery - 1) {
        uint16_t bus_raw = 0;
        bus_ok = ina.readBus(bus_raw);
      }
      if (shunt_ok) {
        peak_current_a = std::max(peak_current_a,
          std::fabs(Ina219Fast::currentA(raw, config_.shunt_ohm)));
      }
      const SafetyEvent event = guard.update({shunt_ok && bus_ok, raw, true, false}, t0 - t_ref);
      if (event != SafetyEvent::NONE) {
        stop_reason = eventName(event);
        break;
      }
      const double remain = t0 + kSessionTickS - env.now();
      if (remain > 0.0) {env.pause(remain);}
    }

    // The measurement loop: one tick is 1 ms; the pulse is rewritten once
    // per kSessionPwmPeriodS and a failed write stops the run on the next
    // tick through the guard's pca_ok.
    if (stop_reason.empty()) {
      loop_started = true;
      for (int i = 0; ; ++i) {
        const double t0 = env.now();
        if (env.stop_requested && env.stop_requested()) {
          stop_reason = "signal";
          break;
        }
        if (have_tick) {max_interval_ms = std::max(max_interval_ms, (t0 - last_t0) * 1000.0);}
        last_t0 = t0;
        have_tick = true;

        uint16_t raw = 0;
        const double t_before = env.now();
        const bool shunt_ok = ina.readShunt(raw);
        const double t_after = env.now();
        max_read_ms = std::max(max_read_ms, (t_after - t_before) * 1000.0);
        bool bus_ok = true;
        bool bus_read = false;
        uint16_t bus_raw = 0;
        if (i % kSessionBusEvery == kSessionBusEvery - 1) {
          bus_read = true;
          bus_ok = ina.readBus(bus_raw);
        }

        const double dt = (i == 0) ? 0.0 : t0 - previous_t0;
        previous_t0 = t0;
        const RampState state = ramp.update(dt);

        const SafetyEvent event = guard.update(
          {shunt_ok && bus_ok, raw, pca_ok, state.plausibility_window}, t0 - t_ref);
        if (event != SafetyEvent::NONE) {
          stop_reason = eventName(event);
          break;
        }
        if (state.done) {
          stop_reason = "completed";
          break;
        }

        if (i % pwm_every == 0) {
          pca_ok = pwm.setPulseUs(state.cmd_us);
          if (pca_ok) {last_cmd_us = state.cmd_us;}
        }

        if (shunt_ok) {
          SessionRow row;
          row.t_s = (t_before + t_after) / 2.0 - t_ref;
          row.stroke_id = state.stroke_id;
          row.direction = state.direction;
          row.cmd_us = last_cmd_us;
          row.shunt_raw = static_cast<int>(static_cast<int16_t>(raw));
          row.has_bus = bus_read && bus_ok;
          row.bus_raw = static_cast<int>(bus_raw);
          rows.push_back(row);
          ++samples;
          peak_current_a = std::max(peak_current_a,
            std::fabs(Ina219Fast::currentA(raw, config_.shunt_ohm)));
        }

        const double remain = t0 + kSessionTickS - env.now();
        if (remain > 0.0) {env.pause(remain);}
      }
    }
  } catch (const std::exception & e) {
    stop_reason = "exception";
    result.error = e.what();
  }

  if (!stop_reason.empty()) {result.stop_reason = stop_reason;}

  // Metrics of the run, before anything can fail on the way out.
  result.samples = samples;
  result.duration_s = have_tick ? (last_t0 - t_ref) : 0.0;
  result.rate_hz = result.duration_s > 0.0 ?
    static_cast<double>(samples) / result.duration_s : 0.0;
  result.max_interval_ms = max_interval_ms;
  result.max_read_ms = max_read_ms;
  result.peak_current_a = peak_current_a;

  // (5) On ANY exit after the preflight, before the files: release, retried
  // once; a release failure is exit code 3 (D-10).
  bool released_ok = pwm.release();
  if (!released_ok) {released_ok = pwm.release();}
  result.released = released_ok;
  if (released_ok) {
    if (env.on_released) {env.on_released();}
  } else {
    result.error = result.error.empty() ? "could not release the servo outputs" :
      (result.error + "; could not release the servo outputs");
  }

  // (6) The CSV and its meta, when the measurement loop had started; a file
  // error is exit code 1.
  bool file_error = false;
  if (loop_started && !config_.out_path.empty()) {
    std::string write_error;
    if (!writeTextFile(config_.out_path, csvText(rows), write_error) ||
      !writeTextFile(config_.out_path + ".meta.json", metaJson(result), write_error)) {
      file_error = true;
      result.error = result.error.empty() ? write_error : (result.error + "; " + write_error);
    }
  }

  if (!result.released) {
    result.exit_code = 3;
  } else if (file_error) {
    result.exit_code = 1;
  } else if (result.stop_reason == "completed") {
    result.exit_code = 0;
  } else if (result.stop_reason == "invalid_config") {
    result.exit_code = 2;
  } else {
    result.exit_code = 1;
  }
  return result;
}

}  // namespace dog_bench

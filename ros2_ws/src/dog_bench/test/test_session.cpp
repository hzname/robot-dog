// Tests for the measurement session of the servo bench tool: the ramp driven
// on a fake bus and a fake PWM output, the CSV and .meta.json of the
// tools/servo_speed contract, the release on every stop reason and the
// SignalGuard (D-08, D-09, D-10). Deterministic: the time comes from a Sim
// advanced by the bus transaction hook and by the injected pause; no clocks,
// no sleeps.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "dog_bench/i2c_bus.hpp"
#include "dog_bench/ina219_fast.hpp"
#include "dog_bench/pwm_out.hpp"
#include "dog_bench/session.hpp"

using namespace dog_bench;

namespace
{

constexpr double kShuntOhm = 0.1;
constexpr int kInaAddr = 0x41;

/// Deterministic fake time: every I2C transaction costs `tx_cost`, every pause
/// costs exactly what was asked, and the pause numbered `extend_pause` is
/// lengthened by `extend_s` (a scheduling stall).
struct Sim
{
  double t{0.0};
  double tx_cost{0.00025};
  int pauses{0};
  int extend_pause{0};
  double extend_s{0.0};

  double now() const {return t;}
  void pause(double seconds)
  {
    ++pauses;
    t += seconds;
    if (extend_pause > 0 && pauses == extend_pause) {t += extend_s;}
  }
};

/// One rig: fake time, fake bus, a configured INA219 and a recording PWM
/// output. The current profile is written into the shunt register by the
/// transaction hook, so a read always sees the value of its own moment.
struct Rig
{
  Sim sim;
  FakeI2cBus bus;
  Ina219Fast sensor{bus, kInaAddr};
  FakePwm pwm{1130.0, 1610.0};
  std::function<double(double)> current_a = [](double) {return 0.35;};

  Rig()
  {
    bus.addDevice(kInaAddr);
    bus.setReg16(kInaAddr, ina::kRegConfig, 0x399F);
    bus.setReg16(kInaAddr, ina::kRegShunt, 3500);
    bus.setReg16(kInaAddr, ina::kRegBus, 1500 << 3);
  }

  /// The shunt register code for a current at the given moment [A].
  uint16_t rawFor(double t) const
  {
    const long counts = std::lround(current_a(t) * kShuntOhm / 10e-6);
    return static_cast<uint16_t>(static_cast<int16_t>(std::clamp<long>(counts, -32768, 32767)));
  }

  /// Install the transaction hook and configure the sensor.
  bool configure()
  {
    bus.setTransactionHook([this]() {
      sim.t += sim.tx_cost;
      bus.setReg16(kInaAddr, ina::kRegShunt, rawFor(sim.t));
    });
    return sensor.configure(ina::kIna219Fast320mv);
  }

  /// The measurement configuration of the plan: 0.1 Ohm, the fast 320 mV
  /// configuration, channel 1, confirmed, one 3.0 rad/s speed.
  SessionConfig config(const std::string & out_path) const
  {
    SessionConfig c;
    c.speeds = {3.0};
    c.shunt_ohm = kShuntOhm;
    c.ina_config = ina::kIna219Fast320mv;
    c.channel = 1;
    c.confirmed = true;
    c.out_path = out_path;
    return c;
  }

  /// The injected environment: Sim time, no signals, no emergency path.
  SessionEnv env()
  {
    SessionEnv e;
    e.now = [this]() {return sim.now();};
    e.pause = [this](double s) {sim.pause(s);};
    e.stop_requested = []() {return false;};
    e.on_armed = []() {};
    e.on_released = []() {};
    return e;
  }
};

/// The directory the session tests write into: DOG_BENCH_TEST_OUT_DIR when
/// set (the plan's verify reads tracer.csv from there), else the gtest temp
/// directory.
std::string outDir()
{
  const char * dir = std::getenv("DOG_BENCH_TEST_OUT_DIR");
  if (dir != nullptr && dir[0] != '\0') {return dir;}
  return testing::TempDir();
}

std::string outPath(const std::string & name) {return outDir() + "/" + name;}

bool fileExists(const std::string & path)
{
  std::ifstream in(path);
  return in.good();
}

std::string readFile(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

/// One CSV row as tools/servo_speed/analyze.py reads it.
struct Row
{
  double t_s{0.0};
  int stroke_id{0};
  int direction{0};
  double cmd_us{0.0};
  int shunt_raw{0};
  bool has_bus{false};
  int bus_raw{0};
};

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> parts;
  std::string current;
  for (const char ch : line) {
    if (ch == ',') {
      parts.push_back(current);
      current.clear();
    } else {
      current.push_back(ch);
    }
  }
  parts.push_back(current);
  return parts;
}

std::vector<Row> readCsv(const std::string & path, bool & header_ok)
{
  std::ifstream in(path);
  std::vector<Row> rows;
  std::string line;
  header_ok = false;
  if (!std::getline(in, line)) {return rows;}
  header_ok = line == kCsvHeader;
  while (std::getline(in, line)) {
    if (line.empty()) {continue;}
    const std::vector<std::string> parts = split(line);
    if (parts.size() != 6) {continue;}
    Row row;
    row.t_s = std::strtod(parts[0].c_str(), nullptr);
    row.stroke_id = static_cast<int>(std::strtol(parts[1].c_str(), nullptr, 10));
    row.direction = static_cast<int>(std::strtol(parts[2].c_str(), nullptr, 10));
    row.cmd_us = std::strtod(parts[3].c_str(), nullptr);
    row.shunt_raw = static_cast<int>(std::strtol(parts[4].c_str(), nullptr, 10));
    row.has_bus = !parts[5].empty();
    if (row.has_bus) {row.bus_raw = static_cast<int>(std::strtol(parts[5].c_str(), nullptr, 10));}
    rows.push_back(row);
  }
  return rows;
}

int countSubstring(const std::string & text, const std::string & needle)
{
  int count = 0;
  for (std::size_t pos = text.find(needle); pos != std::string::npos;
    pos = text.find(needle, pos + needle.size())) {
    ++count;
  }
  return count;
}

/// Every session run that reaches the loop writes both files.
void expectFiles(const std::string & csv)
{
  EXPECT_TRUE(fileExists(csv)) << csv;
  EXPECT_TRUE(fileExists(csv + ".meta.json")) << csv << ".meta.json";
}

void expectNoFiles(const std::string & csv)
{
  EXPECT_FALSE(fileExists(csv)) << csv;
  EXPECT_FALSE(fileExists(csv + ".meta.json")) << csv << ".meta.json";
}

/// A PWM output whose pulse writes fail from the n-th attempt on while the
/// release path stays healthy. FakePwm::failWritesFrom fails release() too,
/// which would turn a pulse fault into exit 3; the pca_error case of
/// EveryStopReasonReleases needs a write fault that spares the release.
class PulseFaultPwm : public PwmOut
{
public:
  PulseFaultPwm(double min_us, double max_us) : min_us_(min_us), max_us_(max_us) {}

  bool preflight() override {armed_ = true; error_.clear(); return true;}
  bool setPulseUs(double us) override
  {
    if (!armed_) {error_ = "setPulseUs before a successful preflight"; return false;}
    if (!(us >= min_us_ && us <= max_us_)) {error_ = "pulse outside the window"; return false;}
    ++writes_;
    if (fail_from_ > 0 && writes_ >= fail_from_) {
      error_ = "injected pulse write failure";
      return false;
    }
    pulses_.push_back(us);
    return true;
  }
  bool release() override
  {
    ++releases_;
    released_ = true;
    error_.clear();
    return true;
  }
  bool released() const override {return released_;}
  std::string error() const override {return error_;}

  void failPulsesFrom(int index) {fail_from_ = index;}
  const std::vector<double> & pulses() const {return pulses_;}
  int releases() const {return releases_;}

private:
  double min_us_;
  double max_us_;
  bool armed_{false};
  bool released_{false};
  int writes_{0};
  int fail_from_{0};
  int releases_{0};
  std::string error_;
  std::vector<double> pulses_;
};

/// A PWM output that never releases: the release-failure path.
class NoReleasePwm : public PwmOut
{
public:
  bool preflight() override {return true;}
  bool setPulseUs(double) override {return true;}
  bool release() override
  {
    ++releases_;
    error_ = "injected release failure";
    return false;
  }
  bool released() const override {return false;}
  std::string error() const override {return error_;}
  int releases() const {return releases_;}

private:
  int releases_{0};
  std::string error_;
};

}  // namespace

TEST(Session, TracerRampToCsvAndMeta)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  const std::string csv = outPath("tracer.csv");
  SessionConfig c = rig.config(csv);
  // analyze.load_meta requires a grid of at least four speeds (plan 01-06)
  // and the tracer recording must be accepted by the analysis tool as it is,
  // so it runs the first four grid speeds instead of the single 3.0.
  c.speeds = {3.0, 3.5, 4.0, 4.5};
  Session session(c);
  const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());

  EXPECT_EQ(r.stop_reason, "completed");
  EXPECT_EQ(r.exit_code, 0);
  EXPECT_TRUE(r.released);
  EXPECT_GT(r.samples, 25000);
  EXPECT_GE(r.rate_hz, 950.0);
  EXPECT_LE(r.rate_hz, 1050.0);
  EXPECT_NEAR(r.peak_current_a, 0.35, 1e-3);

  // Pulses: from the centre, inside the ramp window, both edges reached.
  const std::vector<double> & pulses = rig.pwm.pulses();
  ASSERT_FALSE(pulses.empty());
  EXPECT_NEAR(pulses.front(), 1370.0, 1e-9);
  const double low = 1370.0 - 25.0 * 9.444;   // 1133.9 us
  const double high = 1370.0 + 25.0 * 9.444;  // 1606.1 us
  for (const double us : pulses) {
    EXPECT_GE(us, low - 1e-9);
    EXPECT_LE(us, high + 1e-9);
  }
  EXPECT_NEAR(*std::min_element(pulses.begin(), pulses.end()), low, 1e-9);
  EXPECT_NEAR(*std::max_element(pulses.begin(), pulses.end()), high, 1e-9);

  // The CSV contract of tools/servo_speed/analyze.py.
  bool header_ok = false;
  const std::vector<Row> rows = readCsv(csv, header_ok);
  EXPECT_TRUE(header_ok);
  ASSERT_FALSE(rows.empty());
  for (std::size_t i = 1; i < rows.size(); ++i) {
    EXPECT_GT(rows[i].t_s, rows[i - 1].t_s) << "row " << i;
  }
  std::set<int> ids;
  int last_id = -1;
  bool saw_outside = false;
  for (const Row & row : rows) {
    if (row.stroke_id < 0) {
      EXPECT_EQ(row.stroke_id, -1) << "row id " << row.stroke_id;
      saw_outside = true;
      continue;
    }
    ids.insert(row.stroke_id);
    if (row.stroke_id != last_id) {
      EXPECT_EQ(row.stroke_id, last_id + 1);  // numbered without gaps
      last_id = row.stroke_id;
    }
  }
  EXPECT_TRUE(saw_outside);
  EXPECT_EQ(ids.size(), 40u);
  EXPECT_EQ(*ids.begin(), 0);
  EXPECT_EQ(*ids.rbegin(), 39);
  // Even strokes: direction +1 and the cmd staircase rises.
  for (int id = 0; id < 40; id += 2) {
    double previous = 0.0;
    double first = 0.0;
    double last = 0.0;
    int seen = 0;
    for (const Row & row : rows) {
      if (row.stroke_id != id) {continue;}
      EXPECT_EQ(row.direction, 1) << "stroke " << id;
      if (seen == 0) {first = row.cmd_us;}
      if (seen > 0) {EXPECT_GE(row.cmd_us, previous) << "stroke " << id;}
      previous = row.cmd_us;
      last = row.cmd_us;
      ++seen;
    }
    EXPECT_GT(seen, 0) << "stroke " << id;
    EXPECT_GT(last, first) << "stroke " << id;
  }
  // The bus is read on every 8th row.
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const bool eighth = (i + 1) % static_cast<std::size_t>(kSessionBusEvery) == 0;
    EXPECT_EQ(rows[i].has_bus, eighth) << "row " << i;
  }
  // The meta carries exactly the 11 keys of the plan.
  const std::string meta = readFile(csv + ".meta.json");
  const char * keys[11] = {"shunt_ohm", "us_per_deg", "amp_deg", "center_us", "channel",
    "speeds_rad_s", "strokes_per_speed", "hold_s", "rest_s", "ina_config", "stop_reason"};
  for (const char * key : keys) {
    EXPECT_NE(meta.find(std::string("\"") + key + "\":"), std::string::npos) << key;
  }
  EXPECT_EQ(countSubstring(meta, "\":"), 11);
  EXPECT_NE(meta.find("\"stop_reason\": \"completed\""), std::string::npos);
}

TEST(Session, OvercurrentStopsAndReleases)
{
  Rig rig;
  rig.current_a = [](double t) {return t >= 3.5 ? 2.5 : 0.35;};
  ASSERT_TRUE(rig.configure());
  const std::string csv = outPath("overcurrent.csv");
  const double t_ref = rig.sim.t;
  Session session(rig.config(csv));
  const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());

  EXPECT_EQ(r.stop_reason, "overcurrent");
  EXPECT_EQ(r.exit_code, 1);
  EXPECT_TRUE(r.released);
  EXPECT_GE(rig.pwm.releases(), 1);
  // The trip comes within 60 ms of the jump at t = 3.5 s.
  const double stop_after_jump = t_ref + r.duration_s - 3.5;
  EXPECT_GT(stop_after_jump, 0.0);
  EXPECT_LE(stop_after_jump, 0.06);
  expectFiles(csv);
}

TEST(Session, EveryStopReasonReleases)
{
  // saturation: 3.05 A from t = 1 s trips the 95 % hard limit at once.
  {
    Rig rig;
    rig.current_a = [](double t) {return t >= 1.0 ? 3.05 : 0.35;};
    ASSERT_TRUE(rig.configure());
    const std::string csv = outPath("saturation.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "saturation");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }
  // implausible_shunt: 0.01 A peaks below the 0.03 A floor of the window.
  {
    Rig rig;
    rig.current_a = [](double) {return 0.01;};
    ASSERT_TRUE(rig.configure());
    const std::string csv = outPath("implausible.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "implausible_shunt");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }
  // ina_errors: three consecutive failed shunt reads from loop tick 100 on.
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    int fail_budget = 0;
    bool injected = false;
    rig.bus.setTransactionHook([&rig, &fail_budget, &injected]() {
      rig.sim.t += rig.sim.tx_cost;
      rig.bus.setReg16(kInaAddr, ina::kRegShunt, rig.rawFor(rig.sim.t));
      if (!injected && rig.sim.pauses == kSessionPreRollTicks + 100) {
        injected = true;
        fail_budget = 3;
      }
      if (fail_budget > 0) {
        rig.bus.failRange(rig.bus.transactions() - 1, 1);
        --fail_budget;
      }
    });
    const std::string csv = outPath("ina_errors.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "ina_errors");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }

  // pca_error: the 30th pulse write fails; the release path stays healthy.
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    PulseFaultPwm pwm(1130.0, 1610.0);
    pwm.failPulsesFrom(30);
    const std::string csv = outPath("pca_error.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "pca_error");
    EXPECT_TRUE(r.released);
    EXPECT_GE(pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }
  // tick_overrun: one pause is 60 ms too long.
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    rig.sim.extend_pause = kSessionPreRollTicks + 200;
    rig.sim.extend_s = 0.06;
    const std::string csv = outPath("tick_overrun.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "tick_overrun");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }
  // timeout: the global budget of 5 s ends the run.
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("timeout.csv"));
    c.safety.max_seconds = 5.0;
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "timeout");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(c.out_path);
  }
  // signal: a soft signal arrives at t = 2 s.
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionEnv e = rig.env();
    e.stop_requested = [&rig]() {return rig.sim.t >= 2.0;};
    const std::string csv = outPath("signal.csv");
    Session session(rig.config(csv));
    const SessionResult r = session.run(rig.sensor, rig.pwm, e);
    EXPECT_EQ(r.stop_reason, "signal");
    EXPECT_TRUE(r.released);
    EXPECT_GE(rig.pwm.releases(), 1);
    EXPECT_EQ(r.exit_code, 1);
    expectFiles(csv);
  }
}

TEST(Session, ExceptionReleases)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  const std::string csv = outPath("exception.csv");
  SessionEnv e = rig.env();
  int pause_calls = 0;
  e.pause = [&rig, &pause_calls](double seconds) {
    ++pause_calls;
    if (pause_calls == kSessionPreRollTicks + 10) {
      throw std::runtime_error("injected pause failure");
    }
    rig.sim.pause(seconds);
  };
  Session session(rig.config(csv));
  const SessionResult r = session.run(rig.sensor, rig.pwm, e);

  EXPECT_EQ(r.stop_reason, "exception");
  EXPECT_NE(r.error.find("injected pause failure"), std::string::npos);
  EXPECT_TRUE(r.released);
  EXPECT_GE(rig.pwm.releases(), 1);
  EXPECT_EQ(r.exit_code, 1);
  expectFiles(csv);
}

TEST(Session, RefusesForeignChannel)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  rig.pwm.failPreflight("injected preflight failure");
  const std::string csv = outPath("refused_channel.csv");
  const int tx0 = rig.bus.transactions();
  Session session(rig.config(csv));
  const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());

  EXPECT_EQ(r.stop_reason, "refused_foreign_channel");
  EXPECT_EQ(r.exit_code, 1);
  EXPECT_EQ(r.error, "injected preflight failure");
  EXPECT_TRUE(rig.pwm.pulses().empty());
  EXPECT_EQ(rig.bus.transactions(), tx0);  // the bus is not touched
  expectNoFiles(csv);
}

TEST(Session, RefusesWithoutConfirmation)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  SessionConfig c = rig.config(outPath("refused_confirm.csv"));
  c.confirmed = false;
  const int tx0 = rig.bus.transactions();
  Session session(c);
  const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());

  EXPECT_EQ(r.stop_reason, "refused_not_confirmed");
  EXPECT_EQ(r.exit_code, 1);
  EXPECT_TRUE(rig.pwm.pulses().empty());
  EXPECT_EQ(rig.bus.transactions(), tx0);
  expectNoFiles(c.out_path);
}

TEST(Session, NoPulseBeforeInaAnswers)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  // The sensor goes silent: every later transaction fails.
  rig.bus.failRange(rig.bus.transactions(), 1000000);
  Session session(rig.config(outPath("no_answer.csv")));
  const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());

  EXPECT_EQ(r.stop_reason, "ina_errors");
  EXPECT_TRUE(rig.pwm.pulses().empty());  // not one pulse went out
  EXPECT_GE(rig.pwm.releases(), 1);
  EXPECT_EQ(r.exit_code, 1);
}

TEST(Session, InvalidConfigFailsClosed)
{
  // amp_deg above the hard 30 deg limit
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("bad_amp.csv"));
    c.ramp.amp_deg = 31.0;
    const int tx0 = rig.bus.transactions();
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
    EXPECT_TRUE(rig.pwm.pulses().empty());
    EXPECT_EQ(rig.bus.transactions(), tx0);
    expectNoFiles(c.out_path);
  }
  // empty speeds
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("bad_speeds.csv"));
    c.speeds.clear();
    const int tx0 = rig.bus.transactions();
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
    EXPECT_TRUE(rig.pwm.pulses().empty());
    EXPECT_EQ(rig.bus.transactions(), tx0);
    expectNoFiles(c.out_path);
  }
  // shunt_ohm 0
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("bad_shunt.csv"));
    c.shunt_ohm = 0.0;
    const int tx0 = rig.bus.transactions();
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
    EXPECT_TRUE(rig.pwm.pulses().empty());
    EXPECT_EQ(rig.bus.transactions(), tx0);
    expectNoFiles(c.out_path);
  }
  // channel 16
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("bad_channel.csv"));
    c.channel = 16;
    const int tx0 = rig.bus.transactions();
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
    EXPECT_TRUE(rig.pwm.pulses().empty());
    EXPECT_EQ(rig.bus.transactions(), tx0);
    expectNoFiles(c.out_path);
  }
  // empty clock
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("no_clock.csv"));
    SessionEnv e = rig.env();
    e.now = std::function<double()>{};
    const int tx0 = rig.bus.transactions();
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, e);
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
    EXPECT_TRUE(rig.pwm.pulses().empty());
    EXPECT_EQ(rig.bus.transactions(), tx0);
    expectNoFiles(c.out_path);
  }
  // the configuration wins over the confirmation
  {
    Rig rig;
    ASSERT_TRUE(rig.configure());
    SessionConfig c = rig.config(outPath("bad_and_unconfirmed.csv"));
    c.ramp.amp_deg = 31.0;
    c.confirmed = false;
    Session session(c);
    const SessionResult r = session.run(rig.sensor, rig.pwm, rig.env());
    EXPECT_EQ(r.stop_reason, "invalid_config");
    EXPECT_EQ(r.exit_code, 2);
  }
}

TEST(Session, ReleaseFailureGivesExit3)
{
  Rig rig;
  ASSERT_TRUE(rig.configure());
  NoReleasePwm pwm;
  SessionEnv e = rig.env();
  e.stop_requested = [&rig]() {return rig.sim.t >= 0.5;};
  Session session(rig.config(outPath("release_fail.csv")));
  const SessionResult r = session.run(rig.sensor, pwm, e);

  EXPECT_EQ(r.stop_reason, "signal");
  EXPECT_FALSE(r.released);
  EXPECT_EQ(r.exit_code, 3);
  EXPECT_FALSE(r.error.empty());
  EXPECT_GE(pwm.releases(), 2);  // the retry is part of the contract
}

TEST(SignalGuard, SoftSignalsSetFlagAndRestoreHandlers)
{
  const int signals[8] = {SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGSEGV, SIGABRT, SIGBUS, SIGFPE};
  struct sigaction before[8];
  for (int i = 0; i < 8; ++i) {
    ASSERT_EQ(sigaction(signals[i], nullptr, &before[i]), 0);
  }
  {
    SignalGuard guard;
    EXPECT_TRUE(guard.ok());
    EXPECT_FALSE(guard.stopRequested());
    EXPECT_EQ(guard.lastSignal(), 0);
    for (int i = 0; i < 4; ++i) {
      ASSERT_EQ(raise(signals[i]), 0);
      EXPECT_TRUE(guard.stopRequested());
      EXPECT_EQ(guard.lastSignal(), signals[i]);
    }
  }
  for (int i = 0; i < 8; ++i) {
    struct sigaction after;
    ASSERT_EQ(sigaction(signals[i], nullptr, &after), 0);
    EXPECT_EQ(after.sa_handler, before[i].sa_handler) << "signal " << signals[i];
  }
}

TEST(SignalGuard, FatalSignalsWriteFiveBytesAndExit3)
{
  const int signals[4] = {SIGSEGV, SIGABRT, SIGBUS, SIGFPE};
  for (const int sig : signals) {
    int fds[2];
    ASSERT_EQ(pipe(fds), 0);
    const pid_t pid = fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      ::close(fds[0]);
      SignalGuard guard;
      if (!guard.ok()) {::_exit(97);}
      guard.armEmergency(fds[1]);
      raise(sig);
      ::_exit(98);  // raise must not return
    }
    ::close(fds[1]);
    uint8_t buf[8] = {0};
    const ssize_t n = ::read(fds[0], buf, sizeof(buf));
    ::close(fds[0]);
    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);
    EXPECT_TRUE(WIFEXITED(status)) << "signal " << sig;
    EXPECT_EQ(WEXITSTATUS(status), 3) << "signal " << sig;
    ASSERT_EQ(n, 5) << "signal " << sig;
    EXPECT_EQ(buf[0], 0xFA);
    EXPECT_EQ(buf[1], 0x00);
    EXPECT_EQ(buf[2], 0x00);
    EXPECT_EQ(buf[3], 0x00);
    EXPECT_EQ(buf[4], 0x10);
  }
}

TEST(SignalGuard, DisarmedFatalSignalWritesNothing)
{
  int fds[2];
  ASSERT_EQ(pipe(fds), 0);
  const pid_t pid = fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::close(fds[0]);
    SignalGuard guard;
    guard.armEmergency(fds[1]);
    guard.disarmEmergency();
    raise(SIGSEGV);
    ::_exit(98);
  }
  ::close(fds[1]);
  uint8_t buf[8] = {0};
  const ssize_t n = ::read(fds[0], buf, sizeof(buf));
  ::close(fds[0]);
  int status = 0;
  ASSERT_EQ(waitpid(pid, &status, 0), pid);
  EXPECT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 3);  // still the emergency exit code
  EXPECT_EQ(n, 0);                    // but nothing was written
}

TEST(SignalGuard, EmergencyFdRefusesMissingDevice)
{
  std::string error;
  EXPECT_EQ(openEmergencyFd("/dev/i2c-99", error), -1);
  EXPECT_FALSE(error.empty());
}

TEST(Confirmation, AcceptsOnlyExactYes)
{
  EXPECT_TRUE(isConfirmed("YES"));
  EXPECT_TRUE(isConfirmed("YES\n"));
  EXPECT_TRUE(isConfirmed("YES\r\n"));
  EXPECT_FALSE(isConfirmed("yes"));
  EXPECT_FALSE(isConfirmed("Y"));
  EXPECT_FALSE(isConfirmed(""));
  EXPECT_FALSE(isConfirmed(" YES"));
  EXPECT_FALSE(isConfirmed("YES please"));
}

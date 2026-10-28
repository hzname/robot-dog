// Fall protection, the required IMU and the thermal timeout (phase 4).
//
// The rules under test:
//   * a soft tilt lies the robot down, a hard one latches FALLEN after 0.2 s;
//   * a fallen robot refuses every command except ~/recover;
//   * a silent IMU is a fall: a blind robot must not keep walking;
//   * holding a pose for the thermal timeout lies the robot down and releases
//     the servos only after it is down.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "dog_control/locomotion.hpp"

using dog_control::BodyVelocity;
using dog_control::LocomotionController;
using dog_control::LocomotionParams;
using dog_control::Mode;

namespace
{

constexpr double kDeg = M_PI / 180.0;

LocomotionParams benchParams()
{
  LocomotionParams p;
  p.gait.period = 0.6;
  p.safety_require_imu = false;
  return p;
}

/// Runs the controller for `seconds` in 0.01 s steps, feeding the IMU every step
/// (the IMU is silent only when the test says so).
void run(LocomotionController & c, double seconds, double roll_deg = 0.0,
  double pitch_deg = 0.0, bool imu = true)
{
  const double dt = 0.01;
  for (double t = 0.0; t < seconds; t += dt) {
    if (imu) {
      c.setImuAttitude(roll_deg * kDeg, pitch_deg * kDeg, dt);
    }
    c.update(dt);
  }
}

/// Stands the robot up (and lets the transition finish).
void stand(LocomotionController & c)
{
  ASSERT_TRUE(c.request("stand"));
  run(c, 2.0);
  ASSERT_EQ(c.mode(), Mode::STAND);
}

}  // namespace

// ------------------------------------------------------------------ tilt levels

TEST(SafetyTilt, SoftTiltLiesTheRobotDown)
{
  LocomotionController c(benchParams());
  stand(c);

  run(c, 0.5, 23.0);  // above soft (22), below hard (32)
  EXPECT_EQ(c.mode(), Mode::LYING_DOWN) << "a soft tilt must lie the robot down";
  EXPECT_FALSE(c.fallen());
}

TEST(SafetyTilt, HardTiltLatchesFallenAfterTheDebounce)
{
  LocomotionController c(benchParams());
  stand(c);

  run(c, 0.15, 33.0);  // shorter than the 0.2 s debounce
  EXPECT_FALSE(c.fallen()) << "a short spike must not latch a fall";

  run(c, 0.15, 33.0);  // now past 0.2 s in total
  EXPECT_TRUE(c.fallen());
  EXPECT_EQ(c.mode(), Mode::FALLEN);
  EXPECT_NE(std::string::npos, c.fallReason().find("33")) << c.fallReason();
}

TEST(SafetyTilt, AHardTiltThatEndsInsideTheDebounceDoesNotLatch)
{
  LocomotionController c(benchParams());
  stand(c);

  run(c, 0.1, 33.0);
  run(c, 0.5, 0.0);   // back to level
  run(c, 0.1, 33.0);  // the counter restarted
  EXPECT_FALSE(c.fallen());
}

TEST(SafetyTilt, TheSoftThresholdHasHysteresis)
{
  LocomotionParams p = benchParams();
  LocomotionController c(p);
  stand(c);

  run(c, 0.1, 23.0);  // soft active
  run(c, 0.1, 20.5);  // still above soft - hysteresis (19)
  EXPECT_TRUE(c.mode() == Mode::LYING_DOWN || c.mode() == Mode::LYING ||
    c.mode() == Mode::STAND) << dog_control::modeName(c.mode());
  // Once it is lying the soft level does not fight the operator: a level robot
  // standing again must not be laid down by a stale flag.
  run(c, 2.0, 0.0);
  EXPECT_TRUE(c.request("stand"));
  run(c, 2.0, 20.0);
  EXPECT_EQ(c.mode(), Mode::STAND) << "20 deg is below soft and must not lie the robot down";
}

// ------------------------------------------------------------------ fallen

TEST(SafetyFallen, AFallenRobotRefusesEveryCommand)
{
  LocomotionController c(benchParams());
  stand(c);
  run(c, 0.3, 33.0);
  ASSERT_TRUE(c.fallen());

  EXPECT_FALSE(c.request("stand"));
  EXPECT_FALSE(c.request("trot"));
  EXPECT_FALSE(c.request("greet"));
  EXPECT_NE(std::string::npos, c.lastRefusal().find("recover")) << c.lastRefusal();
  EXPECT_FALSE(c.update(0.01)) << "a fallen controller must not solve joints";
}

TEST(SafetyFallen, RecoverRefusesWhileTheRobotIsStillTilted)
{
  LocomotionController c(benchParams());
  stand(c);
  run(c, 0.3, 33.0);
  ASSERT_TRUE(c.fallen());

  const std::string why = c.recover();
  EXPECT_FALSE(why.empty());
  EXPECT_NE(std::string::npos, why.find("tilt")) << why;
  EXPECT_TRUE(c.fallen());
}

TEST(SafetyFallen, RecoverPutsTheRobotBackOnItsFeet)
{
  LocomotionController c(benchParams());
  stand(c);
  run(c, 0.3, 33.0);
  ASSERT_TRUE(c.fallen());

  run(c, 0.2, 0.0);  // the operator put it level
  EXPECT_EQ(c.recover(), std::string());
  EXPECT_FALSE(c.fallen());
  EXPECT_EQ(c.mode(), Mode::LYING);
  EXPECT_TRUE(c.request("stand"));
  run(c, 2.0, 0.0);
  EXPECT_EQ(c.mode(), Mode::STAND);
}

TEST(SafetyFallen, AFallAsksForTheOutputsToBeReleased)
{
  LocomotionController c(benchParams());
  stand(c);
  ASSERT_FALSE(c.takeRelaxRequest());

  run(c, 0.3, 33.0);
  ASSERT_TRUE(c.fallen());
  EXPECT_TRUE(c.takeRelaxRequest()) << "a fall must release the servos";
  EXPECT_FALSE(c.takeRelaxRequest()) << "the request is consumed once";
}

TEST(SafetyFallen, AnEstopDoesNotEraseTheFall)
{
  LocomotionController c(benchParams());
  stand(c);
  run(c, 0.3, 33.0);
  ASSERT_TRUE(c.fallen());

  c.setEstop(true);   // the node publishes estop for its own fall
  c.setEstop(false);
  EXPECT_TRUE(c.fallen()) << "the fall must outlive the e-stop banner it raised";
}

// ------------------------------------------------------------------ silent IMU

TEST(SafetyImu, ASilentImuFallsTheRobotInTheFloorProfile)
{
  LocomotionParams p = benchParams();
  p.safety_require_imu = true;
  LocomotionController c(p);
  c.setImuCalibrated(true);
  stand(c);

  run(c, 0.5, 0.0, 0.0, false);  // the IMU stopped answering
  EXPECT_TRUE(c.fallen());
  EXPECT_NE(std::string::npos, c.fallReason().find("imu")) << c.fallReason();
}

TEST(SafetyImu, OnTheBenchAnImuThatWasNeverThereIsFine)
{
  LocomotionController c(benchParams());  // bench: require_imu false
  EXPECT_TRUE(c.request("stand"));
  run(c, 5.0, 0.0, 0.0, false);
  EXPECT_FALSE(c.fallen()) << "no IMU at all is normal on the bench";
}

TEST(SafetyImu, OnTheBenchAnImuThatWentSilentIsAFall)
{
  LocomotionController c(benchParams());
  stand(c);            // IMU has been talking all along
  run(c, 0.5, 0.0, 0.0, false);
  EXPECT_TRUE(c.fallen()) << "once the IMU has spoken, silence means it is gone";
}

TEST(SafetyImu, TheFloorProfileRefusesToStandWithoutACalibratedImu)
{
  LocomotionParams p = benchParams();
  p.safety_require_imu = true;
  LocomotionController c(p);

  EXPECT_FALSE(c.request("stand"));
  EXPECT_NE(std::string::npos, c.lastRefusal().find("calibrated")) << c.lastRefusal();

  c.setImuCalibrated(true);
  EXPECT_TRUE(c.request("stand"));
}

// ------------------------------------------------------------------ thermal timeout

TEST(SafetyIdle, HoldingAPoseForTheTimeoutLiesDownAndRelaxesAfterwards)
{
  LocomotionParams p = benchParams();
  p.safety_idle_lie_s = 60.0;  // the shortest deadline the servos tolerate
  LocomotionController c(p);
  stand(c);

  run(c, 57.0);  // 2 s of standing up already counted towards the deadline
  ASSERT_EQ(c.mode(), Mode::STAND) << "the timer must not fire early";
  EXPECT_FALSE(c.takeRelaxRequest());

  run(c, 1.5);
  EXPECT_TRUE(c.mode() == Mode::LYING_DOWN || c.mode() == Mode::LYING);
  // While the robot is still on its way down the servos hold it.
  run(c, 0.1);
  EXPECT_FALSE(c.takeRelaxRequest()) << "the servos hold the robot until it is down";

  run(c, 2.0);  // the lie-down finished
  ASSERT_EQ(c.mode(), Mode::LYING);
  EXPECT_TRUE(c.takeRelaxRequest()) << "once down, the outputs are released";
}

TEST(SafetyIdle, ACommandRestartsTheTimer)
{
  LocomotionParams p = benchParams();
  p.safety_idle_lie_s = 60.0;
  LocomotionController c(p);
  stand(c);

  run(c, 50.0);
  c.setVelocity(BodyVelocity{0.05, 0.0, 0.0});
  run(c, 5.0);
  EXPECT_TRUE(c.mode() == Mode::STAND || c.mode() == Mode::WALK);
  EXPECT_FALSE(c.fallen());
}

TEST(SafetyIdle, APassiveRobotDoesNotCountTime)
{
  LocomotionParams p = benchParams();
  p.safety_idle_lie_s = 60.0;
  LocomotionController c(p);

  run(c, 120.0);  // never stood up
  EXPECT_EQ(c.mode(), Mode::PASSIVE);
}

TEST(SafetyIdle, TheDeadlineIsValidated)
{
  LocomotionParams p = benchParams();
  p.safety_idle_lie_s = 5.0;  // below the 60 s floor
  EXPECT_THROW(LocomotionController{p}, std::invalid_argument);
  p.safety_idle_lie_s = 500.0;  // above 120
  EXPECT_THROW(LocomotionController{p}, std::invalid_argument);
  p.safety_idle_lie_s = 0.0;    // off
  EXPECT_NO_THROW(LocomotionController{p});
}

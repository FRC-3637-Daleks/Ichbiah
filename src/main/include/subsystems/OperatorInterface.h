#pragma once

#include "subsystems/Elevator.h"

#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

namespace OperatorConstants {
constexpr auto kMatchTimeStdError = 20_ms;
constexpr auto kClimbExtendTime = 30_s;
constexpr auto kClimbRetractTime = 1.5_s;

constexpr auto kMinClimbExtendTime = kClimbExtendTime - 2 * kMatchTimeStdError;
constexpr auto kMaxClimbExtendTime = kClimbExtendTime + 2 * kMatchTimeStdError;

constexpr auto kMinClimbRetractTime =
    kClimbRetractTime - 2 * kMatchTimeStdError;
constexpr auto kMaxClimbRetractTime =
    kClimbRetractTime + 2 * kMatchTimeStdError;
} // namespace OperatorConstants

// #pragma GCC diagnostic ignored
//  DO NOT DO THIS, this is a quick comopetition fix
//  namespace TEMP_COMP_VARIABLES {
//  bool s_climkExtended{false};
//  }

class OperatorInterface {

public:
  OperatorInterface();
  double throttle();
  double boolean_slowdown();
  bool slowmode();
  units::meters_per_second_t strafe();
  units::meters_per_second_t fwd();
  units::meters_per_second_t alt_fwd();
  units::revolutions_per_minute_t rot();
  Elevator::Level target_level() { return m_target_level; }

  // Primary Controls
  frc2::Trigger ZeroHeadingTrigger = m_swerveController.Start();
  frc2::Trigger RobotRelativeToggleTrigger = m_swerveController.Back();
  frc2::Trigger ScoreTrigger = m_swerveController.RightBumper();
  frc2::Trigger CancelScoreTrigger = m_swerveController.B();
  frc2::Trigger L1TargetTrigger =
      m_swerveController.POVDown(); // || m_copilotController.POVDown();
  frc2::Trigger L2TargetTrigger =
      m_swerveController.POVLeft(); // || m_copilotController.POVLeft();
  frc2::Trigger L3TargetTrigger =
      m_swerveController.POVRight(); // || m_copilotController.POVRight();
  frc2::Trigger L4TargetTrigger =
      m_swerveController.POVUp(); // || m_copilotController.POVUp();

  frc2::Trigger ElevatorPrePlaceTrigger = m_swerveController.X();
  frc2::Trigger IntakeTrigger =
      m_swerveController.A() || m_copilotController.A();
  frc2::Trigger ClimbToggleTrigger = m_swerveController.Y();

  frc2::Trigger ClimbTimedExtendTrigger{[this]() -> bool {
    auto time = frc::DriverStation::GetMatchTime();
    // return !TEMP_COMP_VARIABLES::s_climkExtended &&
    //        time >= OperatorConstants::kMinClimbExtendTime &&
    //        time <= OperatorConstants::kMaxClimbExtendTime;
    return time >= OperatorConstants::kMinClimbExtendTime &&
           time <= OperatorConstants::kMaxClimbExtendTime;
  }};
  frc2::Trigger ClimbTimedRetractTrigger{[this]() -> bool {
    auto time = frc::DriverStation::GetMatchTime();
    return time >= OperatorConstants::kMinClimbRetractTime &&
           time <= OperatorConstants::kMaxClimbRetractTime;
  }};

  // Manual Override Controls
  frc2::Trigger ElevatorUpTrigger{
      [this] { return m_copilotController.GetLeftY() < -0.5; }};
  frc2::Trigger ElevatorDownTrigger{
      [this] { return m_copilotController.GetLeftY() > 0.5; }};
  frc2::Trigger EndEffectorInTrigger = m_copilotController.RightTrigger();
  frc2::Trigger EndEffectorOutTrigger = m_copilotController.LeftTrigger();
  frc2::Trigger ElevatorIntakeTrigger = m_copilotController.B();
  frc2::Trigger AutoScoreTrigger = m_copilotController.RightBumper();
  frc2::Trigger ClimbUpTrigger = m_copilotController.Y();
  frc2::Trigger ClimbDownTrigger = m_copilotController.X();
  frc2::Trigger L1Manual = m_copilotController.POVDown();
  frc2::Trigger L2Manual = m_copilotController.POVLeft();
  frc2::Trigger L3Manual = m_copilotController.POVRight();
  frc2::Trigger L4Manual = m_copilotController.POVUp();

  frc2::CommandPtr RumbleController(units::second_t time, double intensity);
  frc2::CommandPtr SetTargetLevelCommand(Elevator::Level level);

private:
  frc2::CommandXboxController m_swerveController;
  frc2::CommandXboxController m_copilotController;
  Elevator::Level m_target_level;
  bool IsRed();
};

#include "subsystems/OperatorInterface.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/RobotBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/button/Trigger.h>
#include <iostream>

#include <units/math.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace OperatorConstants {
constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kStrafeDeadband = 0.08;
constexpr double kRotDeadband = .16;
constexpr double kClimbDeadband = 0.08;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kBack;

constexpr auto kMaxTeleopSpeed = 17.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 3.0 * std::numbers::pi * 1_rad_per_s;
} // namespace OperatorConstants
OperatorInterface::OperatorInterface()
    : m_swerveController{OperatorConstants::kSwerveControllerPort},
      m_copilotController{OperatorConstants::kCopilotControllerPort},
      m_target_level{Elevator::L4} {

  L1TargetTrigger.OnTrue(SetTargetLevelCommand(Elevator::L1));
  L2TargetTrigger.OnTrue(SetTargetLevelCommand(Elevator::L2));
  L3TargetTrigger.OnTrue(SetTargetLevelCommand(Elevator::L3));
  L4TargetTrigger.OnTrue(SetTargetLevelCommand(Elevator::L4));
}

double OperatorInterface::throttle() {
  double input = m_swerveController.GetHID().GetRightTriggerAxis();
  double ret = ((-input + 1));
  return ret;
}

double OperatorInterface::boolean_slowdown() {
  if (m_swerveController.GetHID().GetLeftTriggerAxis() > 0.2)
    return 0.3;
  return 1.0;
}

units::meters_per_second_t OperatorInterface::fwd() {
  auto input = frc::ApplyDeadband(m_swerveController.GetHID().GetLeftY(),
                                  OperatorConstants::kStrafeDeadband);
  auto squaredInput = input * std::abs(input);
  auto alliance_flip = IsRed() ? -1 : 1;
  return -OperatorConstants::kMaxTeleopSpeed * squaredInput * alliance_flip *
         throttle() * boolean_slowdown();
}

units::meters_per_second_t OperatorInterface::alt_fwd() {
  auto input = frc::ApplyDeadband(m_swerveController.GetHID().GetRightY(),
                                  OperatorConstants::kStrafeDeadband);
  auto squaredInput = input * std::abs(input);
  auto alliance_flip = IsRed() ? -1 : 1;
  return -OperatorConstants::kMaxTeleopSpeed * squaredInput * alliance_flip *
         throttle() * boolean_slowdown();
}

units::meters_per_second_t OperatorInterface::strafe() {
  auto input = frc::ApplyDeadband(m_swerveController.GetHID().GetLeftX(),
                                  OperatorConstants::kStrafeDeadband);
  auto squaredInput = input * std::abs(input);
  auto alliance_flip = IsRed() ? -1 : 1;
  return -OperatorConstants::kMaxTeleopSpeed * squaredInput * alliance_flip *
         throttle() * boolean_slowdown();
}

units::revolutions_per_minute_t OperatorInterface::rot() {
  auto input = frc::ApplyDeadband(-m_swerveController.GetHID().GetRightX(),
                                  OperatorConstants::kRotDeadband);
  auto squaredInput = input * std::abs(input);
  return OperatorConstants::kMaxTeleopTurnSpeed * squaredInput * throttle() *
         boolean_slowdown();
}

bool OperatorInterface::IsRed() {
  auto m_isRed =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed);
  return m_isRed;
}

frc2::CommandPtr OperatorInterface::RumbleController(units::second_t time,
                                                     double intensity) {
  return frc2::cmd::Run([this, intensity] {
           m_copilotController.SetRumble(
               frc::GenericHID::RumbleType::kBothRumble, intensity);
           m_swerveController.SetRumble(
               frc::GenericHID::RumbleType::kBothRumble, intensity);
         })
      .WithTimeout(time)
      .AndThen([this] {
        m_copilotController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                      0);
        m_swerveController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                     0);
      });
}

frc2::CommandPtr
OperatorInterface::SetTargetLevelCommand(Elevator::Level level) {
  return frc2::cmd::RunOnce([this, level] { m_target_level = level; });
}
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
#pragma once

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
constexpr auto kClimbRetractTime = 1_s;

constexpr auto kMinClimbExtendTime = kClimbExtendTime - 2 * kMatchTimeStdError;
constexpr auto kMaxClimbExtendTime = kClimbExtendTime + 2 * kMatchTimeStdError;

constexpr auto kMinClimbRetractTime =
    kClimbRetractTime - 2 * kMatchTimeStdError;
constexpr auto kMaxClimbRetractTime =
    kClimbRetractTime + 2 * kMatchTimeStdError;
} // namespace OperatorConstants

class OperatorInterface {

public:
  OperatorInterface();
  double throttle();
  units::meters_per_second_t strafe();
  units::meters_per_second_t fwd();
  units::revolutions_per_minute_t rot();
  frc2::Trigger ElevatorL1Trigger = m_copilotController.RightBumper();
  frc2::Trigger ElevatorL2Trigger = m_copilotController.B();
  frc2::Trigger ElevatorL3Trigger = m_copilotController.X();
  frc2::Trigger ElevatorL4Trigger = m_copilotController.Y();
  frc2::Trigger ElevatorUpTrigger{
      [this] { return m_copilotController.GetRightY() > 0.5; }};
  frc2::Trigger ElevatorDownTrigger{
      [this] { return m_copilotController.GetRightY() < -0.5; }};
  frc2::Trigger EndEffectorInTrigger = m_copilotController.RightTrigger();
  frc2::Trigger EndEffectorOutTrigger = m_copilotController.LeftTrigger();
  frc2::Trigger ClimbUpTrigger = m_swerveController.Y();
  frc2::Trigger ClimbDownTrigger = m_swerveController.A();
  frc2::Trigger ElevatorIntakeTrigger = m_copilotController.A();
  frc2::Trigger FollowPathTrigger = m_swerveController.POVDown();
  frc2::Trigger DriveToPoseTrigger = m_swerveController.POVDown();
  frc2::Trigger zeroHeadingTrigger = m_swerveController.Start();
  frc2::Trigger ClimbExtendTrigger{[this]() -> bool {
    auto time = frc::DriverStation::GetMatchTime();
    return time >= OperatorConstants::kMinClimbExtendTime &&
           time <= OperatorConstants::kMaxClimbExtendTime;
  }};
  frc2::Trigger ClimbRetractTrigger{[this]() -> bool {
    auto time = frc::DriverStation::GetMatchTime();
    return time >= OperatorConstants::kMinClimbRetractTime &&
           time <= OperatorConstants::kMaxClimbExtendTime;
  }};

  frc2::CommandPtr RumbleController(units::second_t time, double intensity);

private:
  frc2::CommandXboxController m_swerveController;
  frc2::CommandXboxController m_copilotController;
  bool IsRed();
};
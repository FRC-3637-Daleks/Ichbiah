// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <units/math.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#include <frc/RobotBase.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>

#include <choreo/Choreo.h>

namespace AutoConstants {

constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 6_mps_sq;
constexpr auto kPathMaxAcceleration = 4_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = PracticeModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

constexpr frc::Pose2d desiredPose{0_m, 0_m, 0_deg};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kStrafeDeadband = 0.08;
constexpr double kRotDeadband = .16;
constexpr double kClimbDeadband = 0.08;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr auto kMaxTeleopSpeed = 15.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;

} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 57_ft + 6.875_in;
constexpr auto field_width = 26_ft + 5_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants


RobotContainer::RobotContainer()
  : m_swerveController(OperatorConstants::kSwerveControllerPort)
  {

  fmt::println("made it to robot container");
  // Initialize all of your commands and subsystems here
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Log Match Info
  std::string matchType =
      frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone
          ? ""
          : (frc::DriverStation::GetMatchType() ==
                     frc::DriverStation::MatchType::kElimination
                 ? "Elimination"
                 : (frc::DriverStation::GetMatchType() ==
                            frc::DriverStation::MatchType::kQualification
                        ? "Qualification"
                        : "Practice"));

  std::string alliance =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
           ? "Red"
           : "Blue");

  frc::DataLogManager::Log(
      fmt::format("Playing {} Match {} at {} as {} alliance\n", matchType,
                  frc::DriverStation::GetMatchNumber(),
                  frc::DriverStation::GetEventName(), alliance));

  // Configure the button bindings
  ConfigureBindings();

  // Configure Dashboard
  ConfigureDashboard();

  // Configure Auton.
  ConfigureAuto();

  // Configure routines which one periodically and indefinitely
  ConfigureContinuous();

  frc::DataLogManager::Log(fmt::format("Finished initializing robot."));
}

void RobotContainer::ConfigureBindings() {
    auto throttle = [this]() -> double { 
    double input = m_swerveController.GetHID().GetThrottle();
    double ret = ((-input +1))/2;
    return ret;
  };
  // Configure Swerve Bindings.
  auto fwd = [this, throttle]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        m_swerveController.GetHID().GetY(),
        OperatorConstants::kStrafeDeadband);
    auto squaredInput = input * std::abs(input);
    auto alliance_flip = IsRed()? -1:1;
    return OperatorConstants::kMaxTeleopSpeed 
      * squaredInput
      * alliance_flip
      * throttle();
  };

  auto strafe = [this, throttle]() -> units::meters_per_second_t {
    auto input = frc::ApplyDeadband(
        m_swerveController.GetHID().GetX(),
        OperatorConstants::kStrafeDeadband);
    auto squaredInput = input * std::abs(input);
    auto alliance_flip = IsRed()? -1:1;
    return OperatorConstants::kMaxTeleopSpeed
      * squaredInput
      * alliance_flip
      * throttle();
  };


  auto rot = [this, throttle]() -> units::revolutions_per_minute_t {
    auto input = frc::ApplyDeadband(
        -m_swerveController.GetHID().GetTwist(),
        OperatorConstants::kRotDeadband);
    auto squaredInput = input * std::abs(input);
    return OperatorConstants::kMaxTeleopTurnSpeed
      * squaredInput
      * throttle();
  };

  frc2::CommandPtr testCmd = frc2::cmd::Run([] {
    fmt::print("Test Command\n");
  });

  m_swerve.SetDefaultCommand(
      m_swerve.CustomSwerveCommand(fwd, strafe, rot));
  
  m_swerveController.Button(12).OnTrue(m_swerve.ZeroHeadingCommand());

  m_swerveController.POVDown().WhileTrue(
    m_swerve.DriveToPoseIndefinitelyCommand(AutoConstants::desiredPose));
  
  PathFollower::registerCommand("test", std::move(testCmd));
  try {
    auto traj = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("Square");
    if (traj.has_value())
      m_swerveController.Button(11).WhileTrue(m_swerve.FollowPathCommand(traj.value()));
  } catch (const std::exception &e) {
    fmt::println("Failed to load trajectory Square because of:\n{}", e.what());
  } catch (...) {
    fmt::println("Failed to load trajectory but we don't know why\
    because the choreo devs doesn't understand C++ exception handling");
  }
  
  m_swerveController.Button(1).OnTrue
    (m_superStructure.moveElevatorTo(m_superStructure.m_elevator.L1));

  m_swerveController.Button(2).OnTrue
    (m_superStructure.moveElevatorTo(m_superStructure.m_elevator.L2));

  m_swerveController.Button(3).OnTrue
    (m_superStructure.moveElevatorTo(m_superStructure.m_elevator.L3));

  m_swerveController.Button(4).OnTrue
    (m_superStructure.moveElevatorTo(m_superStructure.m_elevator.L4));

  m_swerveController.Button(5).OnTrue
    (m_superStructure.moveElevatorTo(Elevator::Level::INTAKE));

  m_swerveController.Button(6).WhileTrue(m_endeffector.WhileIn());
  m_swerveController.Button(7).WhileTrue(m_endeffector.WhileOut());

  //Test commands
  m_swerveController.Button(8).WhileTrue(m_superStructure.m_elevator.MoveUp());
  m_swerveController.Button(9).WhileTrue(m_superStructure.m_elevator.MoveDown());

  m_swerveController.Button(10).WhileTrue(m_superStructure.m_endeffector.EffectorIn());
  m_swerveController.Button(13).WhileTrue(m_superStructure.m_endeffector.EffectorContinue());
  m_swerveController.Button(14).WhileTrue(m_superStructure.m_endeffector.EffectorOut());
}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);

  m_superStructure.InitVisualization(
    m_mech.GetRoot("Elevator", 2.0, 0)->Append<frc::MechanismLigament2d>(
      "Base", 0, 0_deg));
  frc::SmartDashboard::PutData("Visualization", &m_mech);
}

void RobotContainer::ConfigureAuto() {}

void RobotContainer::ConfigureContinuous() {
  // These commands are for transmitting data across subsystems

  // FMS info to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
    frc2::cmd::Run([this] {
      m_ros.CheckFMS();
    })
    .IgnoringDisable(true)
  );

  // Odom to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
    frc2::cmd::Run([this] {
      m_ros.PubOdom(
        m_swerve.GetOdomPose(),
        m_swerve.GetChassisSpeed(),
        m_swerve.GetOdomTimestamp());
    })
    .IgnoringDisable(true)
  );

  /* NOTE: It's a little weird to have a command adjust the pose estimate
   * since 2 other commands might observe different pose estimates.
   * Triggers are evaluated before commands, though, so it shouldnt cause
   * any races there.
   */
  // ROS to swerve
  frc2::CommandScheduler::GetInstance().Schedule(
    frc2::cmd::Run([this] {
      m_swerve.SetMapToOdom(m_ros.GetMapToOdom());
    })
    .IgnoringDisable(true)
  );

  if constexpr (frc::RobotBase::IsSimulation()) {
    frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] {
        m_ros.PubSim(m_swerve.GetSimulatedGroundTruth());
      })
      .IgnoringDisable(true)
    );
  }
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}

bool RobotContainer::IsRed()
{
  m_isRed = (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed);

  return m_isRed;
}
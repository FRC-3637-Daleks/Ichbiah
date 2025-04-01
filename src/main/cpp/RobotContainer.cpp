// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

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

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <iostream>

#include <choreo/Choreo.h>

#include "AutoBuilder.h"

namespace AutoConstants {

constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 6_mps_sq;
constexpr auto kPathMaxAcceleration = 4_mps_sq;
// Swerve Constants (NEED TO BE INTEGRATED)
constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

constexpr frc::Pose2d desiredPose{3.22_m, 3.89_m, 0_deg};
} // namespace AutoConstants

namespace OperatorConstants {

constexpr int kCopilotControllerPort = 1;
constexpr int kSwerveControllerPort = 0;

constexpr double kStrafeDeadband = 0.08;
constexpr double kRotDeadband = .16;
constexpr double kClimbDeadband = 0.08;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kBack;

constexpr auto kMaxTeleopSpeed = 15.7_fps;
constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;

} // namespace OperatorConstants

namespace FieldConstants {

constexpr auto field_length = 57_ft + 6.875_in;
constexpr auto field_width = 26_ft + 5_in;
constexpr auto mid_line = field_length / 2;

} // namespace FieldConstants

RobotContainer::RobotContainer()
    : m_vision(
          [this](frc::Pose2d pose, units::second_t timestamp,
                 wpi::array<double, 3U> stdDevs) {
            m_swerve.AddVisionPoseEstimate(pose, timestamp, stdDevs);
          },
          [this]() { return m_swerve.GetPose(); },
          Eigen::Matrix<double, 3, 1>{1.0, 1.0, 1.0},
          [this] { return m_swerve.GetSimulatedGroundTruth(); }) {
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

  // Needed for LEDs
  frc::SmartDashboard::PutString("Aliance Color", alliance);

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
  // PRIMARY CONTROLS
  m_swerve.SetDefaultCommand(m_swerve.CustomSwerveCommand(
      [this] { return m_oi.fwd(); }, [this] { return m_oi.strafe(); },
      [this] { return m_oi.rot(); }));

  auto slow = m_swerve.CustomRobotRelativeSwerveCommand(
      [this] { return 0_mps; }, [this] { return m_oi.strafe() * 0.3; },
      [this] { return m_oi.rot() * 0.3; });

  m_oi.RobotRelativeToggleTrigger.ToggleOnTrue(
      m_swerve.CustomRobotRelativeSwerveCommand(
          [this] { return m_oi.fwd(); }, [this] { return m_oi.strafe(); },
          [this] { return m_oi.rot(); }));

  m_oi.ZeroHeadingTrigger.OnTrue(m_swerve.ZeroHeadingCommand());

  // Auto Elevator
  /* This changes the elevator heights to set a target level, but not actually
   * commanding the elevator Instead, the driver X button (can change this)
   * actually goes to the target level. The target level starts at L4
   * If they want to score on a different level, the copilot OR pilot can
   * press the D-pad buttons to change it.
   */
  std::function<Elevator::Level()> target_selector =
      [this]() -> Elevator::Level { return m_oi.target_level(); };
  m_oi.ElevatorPrePlaceTrigger.WhileTrue(frc2::cmd::Select(
      target_selector,
      std::pair{Elevator::L1, m_superStructure.prePlace(Elevator::L1)},
      std::pair{Elevator::L2, m_superStructure.prePlace(Elevator::L2)},
      std::pair{Elevator::L3, m_superStructure.prePlace(Elevator::L3)},
      std::pair{Elevator::L4, m_superStructure.prePlace(Elevator::L4)}));

  m_oi.ScoreTrigger
      .WhileTrue(
          frc2::cmd::Select(
              target_selector,
              std::pair{Elevator::L1, m_superStructure.prePlace(Elevator::L1)},
              std::pair{Elevator::L2, m_superStructure.prePlace(Elevator::L2)},
              std::pair{Elevator::L3, m_superStructure.prePlace(Elevator::L3)},
              std::pair{Elevator::L4, m_superStructure.prePlace(Elevator::L4)})
              .DeadlineFor(std::move(slow)))
      .OnFalse((m_endeffector.EffectorOut().DeadlineFor(m_elevator.Hold()))
                   .Unless([this, target_selector] {
                     return m_oi.CancelScoreTrigger.Get() ||
                            frc::SmartDashboard::GetString(
                                "Elevator/Target Level", "INTAKE") == "INTAKE";
                                
                   }));

  // Easy on-stop cancel button for everything.
  m_oi.CancelScoreTrigger.OnTrue(m_superStructure.Reset());

  // When not holding the prePlace button, go to collapsed position
  m_elevator.SetDefaultCommand(m_elevator.GoToLevel(Elevator::INTAKE));

  // Driver Auto Score
  /* Example of how to use CustomSwerveCommand to align in arbitrary ways.
     This code makes it so the robot aligns its angle to the nearest coral
     station pose once the driver has held the trigger for half a second */
  m_oi.IntakeTrigger.Debounce(0.5_s).WhileTrue(m_swerve.CustomSwerveCommand(
      [this] { return m_oi.fwd(); }, [this] { return m_oi.strafe(); },
      [this] {
        return ReefAssist::GetNearestCoralStationPose(m_swerve.GetPose())
            .Rotation()
            .Radians();
      }));
  m_oi.IntakeTrigger.OnTrue(m_superStructure.Intake());

  /* Tested in sim. Auto-Aligns robot to nearest branch
   * Test with caution on real robot.
   * If it seems to be consistent, you can tune the ReefAssist values to make
   * it accurate. You can also make 2 separate binds for LEFT/RIGHT if thats
   * easier for drivers.
   *
   * FusePose() runs here to get the latest correction from OPi.
   * Ideally this would run all the time (like it used to), but since its
   * unstable and untested, I've isolated it to a command so we only use its
   * values when we actually are trying to align to something.
   * --E
   */
  //   m_oi.ElevatorPrePlaceTrigger.WhileTrue(AutoBuilder::LineUp(
  //       AutoBuilder::Direction::RIGHT, m_swerve, m_superStructure));
  // frc2::Trigger SavePosTrigger(
  //     [this]() -> bool { return m_superStructure.IsBranchInReach(); });

  m_oi.AutoScoreTrigger.WhileTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
          m_swerve.CustomSwerveCommand(0_mps, 0_mps, 0_rad_per_s),
          m_endeffector.EffectorOut()),
      frc2::cmd::None(), [this]() -> bool {
        return frc::SmartDashboard::GetBoolean("BranchInReach?", false) &&
               frc::SmartDashboard::GetString("Elevator/Target Level", "L1") ==
                   "L4";
      }));

  // SavePosTrigger.OnTrue([this] { reefPose = m_swerve.GetPose(); });
  // Climb
  m_oi.ClimbTimedExtendTrigger.OnTrue(m_climb.ExtendClimb());
  m_oi.ClimbTimedRetractTrigger.OnTrue(m_climb.RetractClimb());

  // MANUAL OVERRIDES
  // Test Commands for Elevator
  m_oi.ElevatorUpTrigger.WhileTrue(m_elevator.MoveUp());
  m_oi.ElevatorDownTrigger.WhileTrue(m_elevator.MoveDown());
  (m_oi.ElevatorDownTrigger || m_oi.ElevatorUpTrigger)
      .OnFalse(m_elevator.Hold());

  m_oi.L1Manual.WhileTrue(
      m_elevator.GoToLevel(Elevator::L1).AndThen(m_elevator.Hold()));
  m_oi.L2Manual.WhileTrue(
      m_elevator.GoToLevel(Elevator::L2).AndThen(m_elevator.Hold()));
  m_oi.L3Manual.WhileTrue(
      m_elevator.GoToLevel(Elevator::L3).AndThen(m_elevator.Hold()));
  m_oi.L4Manual.WhileTrue(
      m_elevator.GoToLevel(Elevator::L4).AndThen(m_elevator.Hold()));

  // End Effector
  m_oi.EndEffectorInTrigger.WhileTrue(m_endeffector.MotorBackwardCommand());
  m_oi.EndEffectorOutTrigger.WhileTrue(m_endeffector.MotorForwardCommand());

  m_oi.ClimbUpTrigger.OnTrue(m_climb.ExtendClimb());
  m_oi.ClimbDownTrigger.OnTrue(m_climb.RetractClimb());

  m_oi.ClimbToggleTrigger.OnTrue(m_climb.ToggleClimbCommand());

  // Rumble
  frc2::Trigger RumbleTrigger([this]() -> bool {
    return (m_endeffector.IsOuterBreakBeamBroken() &&
            m_endeffector.IsInnerBreakBeamBroken()) ||
           frc::SmartDashboard::GetBoolean("Climb/cage intaked?", false) ||
           (m_superStructure.IsBranchInReach() &&
            frc::SmartDashboard::GetString("Elevator/Target Level", "L1") ==
                "L4") ||
           (m_superStructure.IsBranchInReachL23() &&
            frc::SmartDashboard::GetString("Elevator/Target Level", "Intake") !=
                "Intake");
  });
  RumbleTrigger.OnTrue(m_oi.RumbleController(0.25_s, 1));

  frc2::Trigger RumbleScore([this]() -> bool {
    return frc::SmartDashboard::GetBoolean("Rumble?", false);
  });
  RumbleScore.OnTrue(m_oi.RumbleController(0.25_s, 1));
}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutData("Drivebase", &m_swerve);

  frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] {
        frc::SmartDashboard::PutNumber("Target Height", m_oi.target_level());
      }).IgnoringDisable(true));

  // auto pose = m_swerve.GetPose();
  m_swerve.GetField()
      .GetObject("Blue Reef")
      ->SetPoses(FieldConstants::Blue::kReefPoses);
  m_swerve.GetField()
      .GetObject("Blue coral")
      ->SetPoses({FieldConstants::Blue::kLeftCoralStationPose,
                  FieldConstants::Blue::kRightCoralStationPose});
  m_swerve.GetField()
      .GetObject("Red Reef")
      ->SetPoses(FieldConstants::Red::kReefPoses);
  m_swerve.GetField()
      .GetObject("Red coral")
      ->SetPoses({FieldConstants::Red::kLeftCoralStationPose,
                  FieldConstants::Red::kRightCoralStationPose});
  m_swerve.GetField()
      .GetObject("CoralStations")
      ->SetPoses({FieldConstants::Blue::kRightCoralStationPose,
                  FieldConstants::Blue::kLeftCoralStationPose,
                  FieldConstants::Red::kLeftCoralStationPose,
                  FieldConstants::Red::kRightCoralStationPose});

  m_superStructure.InitVisualization(
      m_mech.GetRoot("Elevator", 2.0, 0)
          ->Append<frc::MechanismLigament2d>("Base", 0, 0_deg));

  constexpr auto pipe_thickness = 10;
  constexpr auto pipe_color = frc::Color::kPurple;
  auto reef = m_mech.GetRoot("Reef", 2.65, 0);
  reef->Append<frc::MechanismLigament2d>("Base", 1.2, 90_deg, 4,
                                         frc::Color::kGray)
      ->Append<frc::MechanismLigament2d>("Trough", 1, -75_deg, 4,
                                         frc::Color::kGray)
      ->Append<frc::MechanismLigament2d>("Branch", 3.0, 75_deg, pipe_thickness,
                                         pipe_color);

  reef->Append<frc::MechanismLigament2d>("L2Tip", 2.65, 90_deg, 0,
                                         frc::Color::kBlack)
      ->Append<frc::MechanismLigament2d>("L2", 1.2, -125_deg, pipe_thickness,
                                         pipe_color);
  reef->Append<frc::MechanismLigament2d>("L3Tip", 3.95, 90_deg, 0,
                                         frc::Color::kBlack)
      ->Append<frc::MechanismLigament2d>("L3", 1.2, -125_deg, pipe_thickness,
                                         pipe_color);
  reef->Append<frc::MechanismLigament2d>("L4Tip", 6, 90_deg, 0,
                                         frc::Color::kBlack)
      ->Append<frc::MechanismLigament2d>("L4", 0.85, -180_deg, pipe_thickness,
                                         pipe_color)
      ->Append<frc::MechanismLigament2d>("L4Base", 1.2, 55_deg, pipe_thickness,
                                         pipe_color);

  frc::SmartDashboard::PutData("Visualization", &m_mech);
  frc::SmartDashboard::PutData(&m_chooser);
}

void RobotContainer::ConfigureAuto() {

  threel4auto =
      AutoBuilder::ThreeL4Auto(m_swerve, m_superStructure, m_updateIsRed);
  threel4autoprocessor = AutoBuilder::ThreeL4AutoProcessor(
      m_swerve, m_superStructure, m_updateIsRed);
  onel4startmidauto =
      AutoBuilder::OneL4StartMidAuto(m_swerve, m_superStructure, m_updateIsRed);

  m_chooser.SetDefaultOption("Default Auto: Line-Up with wall and score 3 L4",
                             threel4auto.get());
  m_chooser.AddOption("Line Up with Processor wall and score 3 L4",
                      threel4autoprocessor.get());
  //   m_chooser.AddOption("just drive", drivethingy.get());
  m_chooser.AddOption("One L4 From Middle", onel4startmidauto.get());
}

void RobotContainer::ConfigureContinuous() {
  // These commands are for transmitting data across subsystems

  // FMS info to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] { m_ros.CheckFMS(); }).IgnoringDisable(true));

  // Odom to ROS
  frc2::CommandScheduler::GetInstance().Schedule(
      frc2::cmd::Run([this] {
        m_ros.PubOdom(m_swerve.GetOdomPose(), m_swerve.GetChassisSpeed(),
                      m_swerve.GetOdomTimestamp());
      }).IgnoringDisable(true));

  /* NOTE: It's a little weird to have a command adjust the pose estimate
   * since 2 other commands might observe different pose estimates.
   * Triggers are evaluated before commands, though, so it shouldnt cause
   * any races there.
   */
  // ROS to swerve
  // Commented out until we really trust it
  frc2::CommandScheduler::GetInstance().Schedule(FusePose());

  if constexpr (frc::RobotBase::IsSimulation()) {
    frc2::CommandScheduler::GetInstance().Schedule(
        frc2::cmd::Run([this] {
          m_ros.PubSim(m_swerve.GetSimulatedGroundTruth());
        }).IgnoringDisable(true));
  }
}

frc2::CommandPtr RobotContainer::FusePose() {
  return frc2::cmd::Run([this] {
           if (const auto transform = m_ros.GetMapToOdom();
               transform.has_value()) {
             frc::SmartDashboard::PutNumber("FusePoseTransform/X",
                                            transform.value().X().value());
             frc::SmartDashboard::PutNumber("FusePoseTransform/Y",
                                            transform.value().Y().value());
             if (!(transform.value().X() <= 0.20_m ||
                   transform.value().Y() == 0.20_m))
               m_swerve.SetMapToOdom(transform.value());
           }
         })
      .IgnoringDisable(true);
}

frc2::Command *RobotContainer::GetAutonomousCommand() {
  //   auto k =
  //       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartBargeToReef");
  //   drivethingy = k.has_value()
  //                     ? m_swerve.FollowPathCommand(k.value(),
  //                     m_updateIsRed()) : frc2::cmd::None();
  //   return std::move(drivethingy);
  return m_chooser.GetSelected();
}

frc2::CommandPtr RobotContainer::GetDisabledCommand() {
  return frc2::cmd::None();
}

bool RobotContainer::IsRed() {
  m_isRed =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed);

  return m_isRed;
}
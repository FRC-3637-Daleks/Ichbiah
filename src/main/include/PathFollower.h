#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>

#include <choreo/trajectory/Trajectory.h>

#include "subsystems/Drivetrain.h"

class PathFollower : public frc2::CommandHelper<frc2::Command, PathFollower> {
public:
  using pose_supplier_t = std::function<frc::Pose2d()>;

  using swerve_state_modifier_t =
      std::function<void(std::array<frc::SwerveModuleState, 4>)>;

  using trajectory_t = choreo::Trajectory<choreo::SwerveSample>;

  /**
   * Constructor
   *
   * @param trajectory The trajectory you want to follow
   * @param desiredPoseSupplier A function that returns the desired pose
   * @param subsystem The subsystem used by this command.
   */
  PathFollower(trajectory_t trajectory, Drivetrain &subsystem,
               std::function<bool()> isRed);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  virtual bool IsFinished();

  static inline void registerCommand(std::string name,
                                     std::shared_ptr<frc2::Command> command) {
    GetNamedCommands().emplace(name, command);
  }

  static inline void registerCommand(std::string name,
                                     frc2::CommandPtr &&command) {
    registerCommand(
        name, std::shared_ptr<frc2::Command>(std::move(command).Unwrap()));
  }

  static inline bool hasCommand(std::string name) {
    return GetNamedCommands().contains(name);
  }

  static frc2::Command *getCommand(std::string name);

  static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> &
  GetNamedCommands() {
    return m_namedCommands;
  }

private:
  static std::unordered_map<std::string, std::shared_ptr<frc2::Command>>
      m_namedCommands;
  static std::unordered_map<std::string, frc::Pose2d> m_eventPoses;
  trajectory_t m_trajectory;
  Drivetrain &m_driveSubsystem;
  frc::Timer m_timer;
  frc::Field2d *m_field;
  std::function<bool()> m_isRed;
};

#include "PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <frc2/command/button/Trigger.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(trajectory_t trajectory, Drivetrain &subsystem,
                           std::function<bool()> isRed)
    : m_trajectory{std::move(trajectory)}, m_driveSubsystem{subsystem},
      m_field{&m_driveSubsystem.GetField()}, m_isRed{isRed} {
  AddRequirements(&m_driveSubsystem);
};

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_driveSubsystem.ResetOdometry(
      m_trajectory.GetInitialPose(m_isRed()).value());
  m_field->GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
  auto events = m_trajectory.events;
  for (auto &e : events) {
    m_eventPoses.emplace(
        e.event, m_trajectory.SampleAt(e.timestamp, m_isRed())->GetPose());
  }
}

void PathFollower::Execute() {
  auto currentTime = m_timer.Get();
  if (auto desiredState = m_trajectory.SampleAt(currentTime, m_isRed())) {
    auto desiredPose = desiredState->GetPose();
    auto feedForward = desiredState->GetChassisSpeeds();
    // for (auto &e : m_eventPoses) {
    //   if (m_driveSubsystem.AtPose(e.second)) {
    //     auto command = getCommand(e.first);
    //     command->Schedule();
    //   }
    // } // error yo
    m_driveSubsystem.DriveToPose(desiredPose, feedForward,
                                 {0.0_m, 0.0_m, 0_deg});
  }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
  m_driveSubsystem.Drive({0_mps, 0_mps, 0_rad_per_s});
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose(m_isRed());
  return finalPose.has_value() && m_driveSubsystem.AtPose(finalPose.value()) &&
         m_driveSubsystem.IsStopped();
}

frc2::Command *PathFollower::getCommand(std::string name) {
  frc2::Command *k = GetNamedCommands().find(name)->second.get();
  // CommandPtr(std::unique_ptr<frc::Command>&& command);
  return k;
}

frc2::CommandPtr
Drivetrain::FollowPathCommand(PathFollower::trajectory_t trajectory,
                              std::function<bool()> isRed) {
  return PathFollower{std::move(trajectory), *this, isRed}.ToPtr();
}

std::unordered_map<std::string, std::shared_ptr<frc2::Command>>
    PathFollower::m_namedCommands{};
std::unordered_map<std::string, frc::Pose2d> PathFollower::m_eventPoses{};

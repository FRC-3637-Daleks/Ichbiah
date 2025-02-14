#include "PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(trajectory_t trajectory, Drivetrain &subsystem)
    : m_trajectory{std::move(trajectory)}, m_driveSubsystem{subsystem},
      m_field{&m_driveSubsystem.GetField()} {
  AddRequirements(&m_driveSubsystem);
};

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_field->GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
  std::cout << "Initializing " << m_trajectory.name << std::endl;
}

void PathFollower::Execute() {
  /**
   * Visvam's TODO list:
   *
   * Either make a child command or add something to iterate through the events
   * and running the commands for them when you pass their timestamp.
   *
   * Maybe a map? <std::string, lambda function> idk just thing about it
   */

  auto currentTime = m_timer.Get();
  std::cout << "Running!!\n";
  if (auto desiredState =
          m_trajectory.SampleAt(currentTime, /* mirror */ false)) {
    auto desiredPose = desiredState->GetPose();
    auto feedForward = desiredState->GetChassisSpeeds();
    m_driveSubsystem.DriveToPose(desiredPose, feedForward,
                                 {0.0_m, 0.0_m, 0_deg});
  }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);

  std::cout << "Finishing " << m_trajectory.name << std::endl;
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose();
  if (!finalPose.has_value()) {
    std::cout << "No Final Pose for " << m_trajectory.name << "\n";
  }
  return finalPose.has_value() && m_driveSubsystem.AtPose(finalPose.value()) &&
         m_driveSubsystem.IsStopped();
}

frc2::CommandPtr
Drivetrain::FollowPathCommand(PathFollower::trajectory_t trajectory) {
  return PathFollower{std::move(trajectory), *this}.ToPtr();
}
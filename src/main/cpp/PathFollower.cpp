#include "PathFollower.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/math.h>

#include <iostream>
#include <numbers>
#include <random>

PathFollower::PathFollower(
  trajectory_t trajectory, Drivetrain &subsystem) 
  : m_trajectory{std::move(trajectory)}, 
    m_driveSubsystem{subsystem},
    m_field{&m_driveSubsystem.GetField()}
{
    AddRequirements(&m_driveSubsystem);
};

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_field->GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
  auto events = m_trajectory.events;
  for(auto &e : events) {
    m_eventPoses.emplace(
      e.event, m_trajectory.SampleAt(e.timestamp, false)
      ->GetPose());
    }
}

void PathFollower::Execute() {
    auto currentTime = m_timer.Get();
    std::cout << "Running!!\n";
    if (auto desiredState = m_trajectory.SampleAt(currentTime, /* mirror */ false)) {
      auto desiredPose = desiredState->GetPose();
      auto feedForward = desiredState->GetChassisSpeeds();
      for(auto &e : m_eventPoses) {
        if(m_driveSubsystem.AtPose(e.second)){
          auto cmd = getCommand(e.first);
          cmd.Schedule();
        }
      } //error yo
      m_driveSubsystem.DriveToPose(desiredPose, feedForward, {0.0_m, 0.0_m, 0_deg});
    }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose();
  return finalPose.has_value() && 
        m_driveSubsystem.AtPose(finalPose.value()) && 
        m_driveSubsystem.IsStopped();
}

 frc2::CommandPtr PathFollower::getCommand(std::string name) {
  frc2::Command* k = GetNamedCommands().find(name)->second.get();
  //CommandPtr(std::unique_ptr<frc::Command>&& command);
    return frc2::CommandPtr(std::make_unique<frc2::Command>(*k));
}

frc2::CommandPtr Drivetrain::FollowPathCommand(
  PathFollower::trajectory_t trajectory) {
  return PathFollower{std::move(trajectory), *this}.ToPtr();
}
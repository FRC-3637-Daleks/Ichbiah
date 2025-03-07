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
#include <tuple>

PathFollower::PathFollower(trajectory_t trajectory, Drivetrain &subsystem,
                           bool isRed)
    : m_trajectory{std::move(trajectory)}, m_driveSubsystem{subsystem},
      m_isRed{isRed}, m_field{&m_driveSubsystem.GetField()} {
  AddRequirements(&m_driveSubsystem);
};

void PathFollower::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_field->GetObject("Trajectory")->SetPoses(m_trajectory.GetPoses());
  auto events = m_trajectory.events;
  for (auto &e : events) {
    m_eventPoses.emplace(e.event,
                         m_trajectory.SampleAt(e.timestamp, false)->GetPose());
  }
}

void PathFollower::Execute() {
  auto currentTime = m_timer.Get();
  if (auto desiredState =
          m_trajectory.SampleAt(currentTime, /* mirror */ m_isRed)) {
    auto desiredPose = desiredState->GetPose();
    auto feedForward = desiredState->GetChassisSpeeds();
    if (!m_eventPoses.empty()) {
      auto it = m_eventPoses.begin();
      if (m_driveSubsystem.AtPose(it->second, {.1_m, .1_m, 20_deg})) {
        auto command = getCommand(it->first);
        std::cout << "Running!!\n";
        command->Schedule();
        m_eventPoses.erase(it); // Remove the executed event
      }
    }
    if (!m_stopPoints.empty()) {
      auto point = m_stopPoints.front();
      auto time = m_timer.Get();
      if (m_driveSubsystem.AtPose(point.first.GetPose()) &&
          m_driveSubsystem.IsStopped()) {
        m_driveSubsystem.CustomSwerveCommand(0_mps, 0_mps, 0_rad_per_s)
            .Until([this, point, time]() -> bool {
              return m_timer.Get() - time >= point.second;
            });
        m_stopPoints.erase(m_stopPoints.begin());
      }
    }
    m_driveSubsystem.DriveToPose(desiredPose, feedForward,
                                 {0.0_m, 0.0_m, 0_deg});
  }
}

void PathFollower::End(bool interrupted) {
  m_timer.Stop();
  m_field->GetObject("Trajectory")->SetPose(100_m, 100_m, 0_deg);
}

// void PathFollower::setStopPoints(int splitPoint, units::second_t stopTime,
//                                  units::second_t offset) {
//   auto time = m_trajectory.GetSplit(splitPoint)
//                   .value()
//                   .GetFinalSample()
//                   .value()
//                   .timestamp +
//               offset;
//   auto splitState = m_trajectory.SampleAt(time, this->m_isRed);
//   if (splitState.has_value()) {
//     for (int i = 0; i < m_stopPoints.size(); i++) {
//       if (splitState.value().timestamp <
//           std::get<0>(m_stopPoints.at(i)).timestamp) {
//         m_stopPoints.insert(m_stopPoints.begin() + i,
//                             std::make_pair(splitState.value(), stopTime));
//         break;
//       }
//     }
//   }
// }

bool PathFollower::IsFinished() {
  auto finalPose = m_trajectory.GetFinalPose();
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
                              bool isRed) {
  return PathFollower{std::move(trajectory), *this, std::move(isRed)}.ToPtr();
}

std::unordered_map<std::string, std::shared_ptr<frc2::Command>>
    PathFollower::m_namedCommands{};
std::unordered_map<std::string, frc::Pose2d> PathFollower::m_eventPoses{};
std::vector<std::pair<choreo::SwerveSample, units::second_t>>
    PathFollower::m_stopPoints{};
#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/SuperStructure.h"

#include <choreo/Choreo.h>

#include <functional>

/**
 * Helper Namespace that Builds Autos
 *
 * As of right now, everything is hard-coded for comp but ideally it will
 * integrate with choreo.
 */

namespace AutoBuilder {

enum Direction { NONE, LEFT, RIGHT };

constexpr auto kLineUpSpeed = 0.32_mps;

auto isRed = []() -> bool {
  return (frc::DriverStation::GetAlliance() ==
          frc::DriverStation::Alliance::kRed);
};

// Left Side Paths
auto IntakeToReefClose =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose");
auto IntakeToReefClose2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose2");
auto ReefCloseToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefCloseToIntake");
auto ReefClose2ToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefClose2ToIntake");
auto ReefFarToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefFarToIntake");
auto StartBargeToReef =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartBargeToReef");

// Center Path
auto StartToReefMid =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReefMid");

// Right Side Paths
auto IntakeToReefCloseProcessor =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
        "IntakeToReefCloseProcessor");
auto IntakeToReefClose2Processor =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
        "IntakeToReefClose2Processor");
auto ReefCloseToIntakeProcessor =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
        "ReefCloseToIntakeProcessor");
auto ReefFarToIntakeProcessor =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
        "ReefFarProcessorToIntake");
auto StartProcessorToReef =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
        "StartProcessorToReef");

namespace util {

inline frc2::CommandPtr LineUp(Direction direction, Drivetrain &swerve,
                               SuperStructure &superstructure) {
  if (direction == Direction::LEFT) {
    return frc2::cmd::Sequence(
        (swerve.CustomRobotRelativeSwerveCommand([] { return 0_mps; },
                                                 [] { return kLineUpSpeed; },
                                                 [] { return 0_rad_per_s; }))
            .Until(
                [&superstructure] { return superstructure.IsBranchInReach(); }),
        swerve.CustomRobotRelativeSwerveCommand([] { return 0_mps; },
                                                [] { return 0_mps; },
                                                [] { return 0_rad_per_s; }));
  }
  if (direction == Direction::RIGHT) {
    return frc2::cmd::Sequence(
        (swerve.CustomRobotRelativeSwerveCommand([] { return 0_mps; },
                                                 [] { return -kLineUpSpeed; },
                                                 [] { return 0_rad_per_s; }))
            .Until(
                [&superstructure] { return superstructure.IsBranchInReach(); }),
        swerve.CustomRobotRelativeSwerveCommand([] { return 0_mps; },
                                                [] { return 0_mps; },
                                                [] { return 0_rad_per_s; }));
  }
  return swerve.CustomRobotRelativeSwerveCommand(
      [] { return 0_mps; }, [] { return 0_mps; }, [] { return 0_rad_per_s; });
}

bool IsCloseToGoal(units::meter_t tolerance,
                   std::optional<choreo::Trajectory<choreo::SwerveSample>> traj,
                   Drivetrain &swerve) {
  auto finalPose = traj.value().GetFinalPose(isRed());
  if (!finalPose.has_value())
    return false;
  auto dist =
      swerve.GetPose().Translation().Distance(finalPose.value().Translation());
  return dist <= tolerance;
}

inline frc2::CommandPtr AutoScore(Elevator::Level level, Direction direction,
                                  Drivetrain &swerve,
                                  SuperStructure &superstructure,
                                  units::second_t timeout = 1.5_s) {
  return frc2::cmd::Sequence(
      superstructure.m_elevator.GoToLevel(level),
      frc2::cmd::Deadline(LineUp(direction, swerve, superstructure),
                          superstructure.m_elevator.Hold())
          .WithTimeout(timeout),
      superstructure.m_endeffector.EffectorOut());
}

inline frc2::CommandPtr
GoToScore(Elevator::Level level,
          std::optional<choreo::Trajectory<choreo::SwerveSample>> traj,
          Drivetrain &swerve, SuperStructure &superstructure,
          units::second_t timeout = 2.2_s) {
  return frc2::cmd::Parallel(
             swerve.FollowPathCommand(traj.value(), isRed),
             frc2::cmd::WaitUntil([traj, &swerve] {
               return IsCloseToGoal(1_m, traj, swerve);
             }).AndThen(superstructure.m_elevator.GoToLevel(level)))
      .WithTimeout(timeout);
}

inline frc2::CommandPtr
AutoIntake(std::optional<choreo::Trajectory<choreo::SwerveSample>> traj,
           Drivetrain &swerve, SuperStructure &superstructure,
           units::second_t timeout = 4.0_s) {
  return frc2::cmd::Parallel(swerve.FollowPathCommand(traj.value(), isRed),
                             superstructure.Intake())
      .WithTimeout(timeout);
}

}; // namespace util

frc2::CommandPtr LeftThreeL4Auto(Drivetrain &swerve,
                                 SuperStructure &superstructure) {
  return frc2::cmd::Sequence(
      util::GoToScore(Elevator::Level::L4, StartBargeToReef, swerve,
                      superstructure),
      util::AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve,
                      superstructure),
      util::AutoIntake(ReefFarToIntake, swerve, superstructure),
      util::GoToScore(Elevator::Level::L4, IntakeToReefClose, swerve,
                      superstructure),
      util::AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve,
                      superstructure, 1.7_s),
      util::AutoIntake(ReefCloseToIntake, swerve, superstructure),
      util::GoToScore(Elevator::Level::L4, IntakeToReefClose2, swerve,
                      superstructure, 5_s),
      util::AutoScore(Elevator::Level::L4, Direction::LEFT, swerve,
                      superstructure),
      util::AutoIntake(ReefClose2ToIntake, swerve, superstructure));
}

frc2::CommandPtr RightThreeL4Auto(Drivetrain &swerve,
                                  SuperStructure &superstructure) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartProcessorToReef.value(), isRed),
      util::AutoScore(Elevator::Level::L4, Direction::LEFT, swerve,
                      superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefFarToIntakeProcessor.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefCloseProcessor.value(), isRed),
      util::AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve,
                      superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefCloseToIntakeProcessor.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose2Processor.value(), isRed),
      util::AutoScore(Elevator::Level::L4, Direction::LEFT, swerve,
                      superstructure));
}

frc2::CommandPtr CenterOneL4Auto(Drivetrain &swerve,
                                 SuperStructure &superstructure) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartToReefMid.value(), isRed),
      util::AutoScore(Elevator::Level::L4, Direction::LEFT, swerve,
                      superstructure));
}
}; // namespace AutoBuilder
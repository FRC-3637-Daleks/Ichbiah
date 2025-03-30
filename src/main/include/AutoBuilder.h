#pragma once

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

constexpr auto kLineUpSpeed = 0.25_mps;

// Left Side Paths
auto IntakeToReefClose =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose");
auto IntakeToReefClose2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose2");
auto ReefCloseToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefCloseToIntake");
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

inline frc2::CommandPtr AutoScore(Elevator::Level level, Direction direction,
                                  Drivetrain &swerve,
                                  SuperStructure &superstructure) {
  return frc2::cmd::Sequence(
      superstructure.m_elevator.GoToLevel(level),
      frc2::cmd::Parallel(superstructure.m_elevator.Hold(),
                          LineUp(direction, swerve, superstructure))
          .WithTimeout(2.5_s),
      superstructure.m_endeffector.EffectorOut());
}

frc2::CommandPtr ThreeL4Auto(Drivetrain &swerve, SuperStructure &superstructure,
                             std::function<bool()> isRed) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartBargeToReef.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefFarToIntake.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::LEFT, swerve, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefCloseToIntake.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose2.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve, superstructure));
}

frc2::CommandPtr ThreeL4AutoProcessor(Drivetrain &swerve,
                                      SuperStructure &superstructure,
                                      std::function<bool()> isRed) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartProcessorToReef.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::LEFT, swerve, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefFarToIntakeProcessor.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefCloseProcessor.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::RIGHT, swerve, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefCloseToIntakeProcessor.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose2Processor.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::LEFT, swerve, superstructure));
}

frc2::CommandPtr OneL4StartMidAuto(Drivetrain &swerve,
                                   SuperStructure &superstructure,
                                   std::function<bool()> isRed) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartToReefMid.value(), isRed),
      AutoScore(Elevator::Level::L4, Direction::LEFT, swerve, superstructure));
}
}; // namespace AutoBuilder
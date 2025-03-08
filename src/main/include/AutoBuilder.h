#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/SuperStructure.h"

#include <choreo/Choreo.h>

/**
 * Helper Namespace that Builds Autos
 *
 * As of right now, everything is hard-coded for comp but ideally it will
 * integrate with choreo.
 */

namespace AutoBuilder {

auto IntakeToReefClose =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose");
auto IntakeToReefClose2 =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose2");
auto ReefCloseToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefCloseToIntake");
auto ReefFarToIntake =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefFarToIntake");
auto StartToReef =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReef");
auto StartToReefMid =
    choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReefMid");

inline frc2::CommandPtr AutoScore(Elevator::Level level,
                                  SuperStructure &superstructure) {
  return frc2::cmd::Sequence(superstructure.prePlace(level),
                             superstructure.Score());
}

inline frc2::CommandPtr
ThreeL4Auto(Drivetrain &swerve, SuperStructure &superstructure, bool isRed) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartToReef.value(), isRed),
      AutoScore(Elevator::Level::L4, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefFarToIntake.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose.value(), isRed),
      AutoScore(Elevator::Level::L4, superstructure),
      frc2::cmd::Parallel(
          swerve.FollowPathCommand(ReefCloseToIntake.value(), isRed),
          superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose2.value(), isRed),
      AutoScore(Elevator::Level::L4, superstructure));
}

inline frc2::CommandPtr OneL4StartMidAuto(Drivetrain &swerve,
                                          SuperStructure &superstructure,
                                          bool isRed) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartToReefMid.value(), isRed),
      AutoScore(Elevator::Level::L4, superstructure));
}
}; // namespace AutoBuilder
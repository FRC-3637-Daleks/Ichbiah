#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"
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

inline frc2::CommandPtr ThreeL4Auto(Drivetrain &swerve,
                                    SuperStructure &superstructure) {
  return frc2::cmd::Sequence(
      swerve.FollowPathCommand(StartToReef.value()),
      superstructure.prePlace(superstructure.m_elevator.L4),
      superstructure.Score(),
      frc2::cmd::Race(swerve.FollowPathCommand(ReefFarToIntake.value()),
                      superstructure.Intake()),
      swerve.FollowPathCommand(IntakeToReefClose.value()),
      superstructure.prePlace(superstructure.m_elevator.L4),
      superstructure.Score());
}

}; // namespace AutoBuilder
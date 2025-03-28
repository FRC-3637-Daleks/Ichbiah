#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/ReefAssist.h"
#include "subsystems/SuperStructure.h"

#include <choreo/Choreo.h>

#include <functional>

/**
 * Helper Namespace that Builds Autos
 *
 * As of right now, everything is hard-coded for comp but ideally it will
 * integrate with choreo.
 */

class AutoBuilder {
  using trajectory_t = choreo::Trajectory<choreo::SwerveSample>;
  /*
  const auto IntakeToReefClose =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose");
  auto IntakeToReefClose2 =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
          "IntakeToReefClose2");
  auto ReefCloseToIntake =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefCloseToIntake");
  auto ReefFarToIntake =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefFarToIntake");
  auto StartToReef =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReef");
  auto StartToReefMid =
      choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReefMid");
    */
public:
  AutoBuilder(Drivetrain &swerve, SuperStructure &superstructure);

public:
  frc2::CommandPtr AutoScore(frc::Pose2d scoring_pose);
  bool isRed();

  // path should have choreo splits at each segment
  // each split will alternate between driving to the reef and driving to a
  // coral station.
  // likewise, robot will sequence scoring and intaking actions between each of
  // these segments
  frc2::CommandPtr MultiL4Auto(trajectory_t path);

  // Full robot-relative, doesn't depend on odom or localization
  // Drives forward at 1_mps for 2.5_s and then scores at L4.
  // Do your best to align
  frc2::CommandPtr SimpleAuto();

private:
  Drivetrain &m_swerve;
  SuperStructure &m_superstructure;
}; // namespace AutoBuilder
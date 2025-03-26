#include "AutoBuilder.h"

#include "PathFollower.h"

#include <fmt/ranges.h>
#include <frc/DriverStation.h>

AutoBuilder::AutoBuilder(Drivetrain &swerve, SuperStructure &superstructure)
    : m_swerve(swerve), m_superstructure(superstructure) {}

bool AutoBuilder::isRed() {
  return frc::DriverStation::GetAlliance() ==
         frc::DriverStation::Alliance::kRed;
}

frc2::CommandPtr AutoBuilder::AutoScore(frc::Pose2d scoring_pose) {
  // line up and prepare elevator in parallel. When both are done, score.
  return m_swerve.DriveToPoseIndefinitelyCommand(scoring_pose, 2_s)
      .RaceWith(m_superstructure.Score(Elevator::L4));
}

frc2::CommandPtr AutoBuilder::MultiL4Auto(trajectory_t path) {
  std::vector<frc2::CommandPtr> commands;
  for (unsigned splitInd = 0; splitInd < path.splits.size(); splitInd++) {
    if (const auto segment = path.GetSplit(splitInd)) {
      if (splitInd % 2 == 0) { // even splits are driving to the reef
        const auto scoring_pose = segment.value().GetFinalPose(isRed()).value();

        // allows path drawing to be sloppy
        const auto snapped_scoring_pose =
            ReefAssist::getNearestScoringPose(scoring_pose);

        commands.emplace_back(frc2::cmd::Print("Driving to reef..."));
        commands.emplace_back(
            m_swerve.FollowPathCommand(segment.value(), isRed())
                .Until([this, scoring_pose] { // start AutoScore early
                  return m_swerve.AtPose(scoring_pose, {0.5_m, 0.5_m, 10_deg});
                }));
        commands.emplace_back(frc2::cmd::Print("Scoring on reef..."));
        commands.emplace_back(AutoScore(snapped_scoring_pose));
      } else { // odd splits are driving to a coral station
        const auto intake_pose = segment.value().GetFinalPose(isRed()).value();

        commands.emplace_back(frc2::cmd::Print("Driving to intake..."));
        // intake along with travel so that we retract elevator asap
        commands.emplace_back(m_superstructure.Intake().DeadlineFor(
            m_swerve.FollowPathCommand(segment.value(), isRed())
                .AndThen(
                    m_swerve.DriveToPoseIndefinitelyCommand(intake_pose))));
      }
    } else { // path split returned error
      break;
    }
  }

  commands.emplace_back(frc2::cmd::Print("Auton complete!"));
  return frc2::cmd::Sequence(std::move(commands));
}

frc2::CommandPtr AutoBuilder::SimpleAuto() {
  return m_swerve.CustomRobotRelativeSwerveCommand(1_mps, 0_mps, 0_deg_per_s)
      .WithTimeout(2.0_s)
      .AndThen(m_swerve.Stop())
      .AndThen(m_superstructure.Score(Elevator::L4));
}
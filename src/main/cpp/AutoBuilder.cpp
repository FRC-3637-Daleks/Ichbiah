// #include "AutoBuilder.h"

// #include "PathFollower.h"
// #include "subsystems/Drivetrain.h"
// #include "subsystems/SuperStructure.h"

// #include <choreo/Choreo.h>

// inline frc2::CommandPtr AutoBuilder::ThreeL4Auto(Drivetrain &swerve,
//                                           SuperStructure &superstructure) {
//   auto IntakeToReefClose =
//       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("IntakeToReefClose");
//   auto IntakeToReefClose2 =
//       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(
//           "IntakeToReefClose2");
//   auto ReefCloseToIntake =
//       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefCloseToIntake");
//   auto ReefFarToIntake =
//       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("ReefFarToIntake");
//   auto StartToReef =
//       choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("StartToReef");

//   return frc2::cmd::Sequence(
//       swerve.FollowPathCommand(StartToReef.value()),
//       superstructure.prePlace(superstructure.m_elevator.L4),
//       superstructure.Score(),
//       frc2::cmd::Race(swerve.FollowPathCommand(ReefFarToIntake.value()),
//                       superstructure.Intake()));
// }
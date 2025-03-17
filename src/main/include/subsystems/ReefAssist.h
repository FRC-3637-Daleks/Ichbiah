#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/math.h>

namespace FieldConstants {
constexpr units::foot_t kFieldLength = 57_ft + 5.5_in;
constexpr units::foot_t kFieldWidth = 26_ft + 5_in;
constexpr units::foot_t kReefDiameter = 5_ft + 5.5_in;
enum Reef { A, B, C, D, E, F, G };
enum Side { LEFT, RIGHT };
constexpr frc::Transform2d kSideTransforms[] = {
    frc::Transform2d{0_in, 6.47_in, 0_deg},
    frc::Transform2d{0_in, -6.47_in, 0_deg}};

// Transform where coral lands relative to robot center when scoring
// Guessing 15in for frame, 2.5in for bumpers, 1.5in for middle of pipe to get
// 19in in front.
// Guessing 12.5in left of robot center
constexpr frc::Transform2d kEndEffectorTransform{19_in, 16.5_in, 0_deg};
constexpr frc::Transform2d kIntakeTransform{19_in, 0_in, 90_deg};

namespace Blue {
constexpr frc::Translation2d kReefCenter{12_ft + (kReefDiameter / 2),
                                         kFieldWidth / 2};

constexpr frc::Pose2d kReefAPose{kReefCenter.X() - kReefDiameter / 2,
                                 kReefCenter.Y(), frc::Rotation2d(0_deg)};
constexpr frc::Pose2d kReefBPose = kReefAPose.RotateAround(kReefCenter, 60_deg);
constexpr frc::Pose2d kReefCPose =
    kReefAPose.RotateAround(kReefCenter, 120_deg);
constexpr frc::Pose2d kReefDPose =
    kReefAPose.RotateAround(kReefCenter, 180_deg);
constexpr frc::Pose2d kReefEPose =
    kReefAPose.RotateAround(kReefCenter, 240_deg);
constexpr frc::Pose2d kReefFPose =
    kReefAPose.RotateAround(kReefCenter, 300_deg);
constexpr frc::Pose2d kReefGPose =
    kReefAPose.RotateAround(kReefCenter, 360_deg);

constexpr frc::Pose2d kReefPoses[] = {kReefAPose, kReefBPose, kReefCPose,
                                      kReefDPose, kReefEPose, kReefFPose,
                                      kReefGPose};

constexpr frc::Pose2d kRightCoralStationPose{(38_in * units::math::cos(36_deg)),
                                             (38_in * units::math::sin(36_deg)),
                                             54_deg};
constexpr frc::Pose2d kLeftCoralStationPose{
    38_in * units::math::cos(36_deg),
    kFieldWidth - 38_in * units::math::sin(36_deg), -54_deg};
} // namespace Blue

namespace Red {
constexpr frc::Translation2d kReefCenter = Blue::kReefCenter.RotateAround(
    {kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefAPose =
    Blue::kReefAPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefBPose =
    Blue::kReefBPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefCPose =
    Blue::kReefCPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefDPose =
    Blue::kReefDPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefEPose =
    Blue::kReefEPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefFPose =
    Blue::kReefFPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kReefGPose =
    Blue::kReefGPose.RotateAround({kFieldLength / 2, kFieldWidth / 2}, 180_deg);

constexpr frc::Pose2d kReefPoses[] = {kReefAPose, kReefBPose, kReefCPose,
                                      kReefDPose, kReefEPose, kReefFPose,
                                      kReefGPose};

constexpr frc::Pose2d kRightCoralStationPose =
    Blue::kRightCoralStationPose.RotateAround(
        {kFieldLength / 2, kFieldWidth / 2}, 180_deg);
constexpr frc::Pose2d kLeftCoralStationPose =
    Blue::kLeftCoralStationPose.RotateAround(
        {kFieldLength / 2, kFieldWidth / 2}, 180_deg);

} // namespace Red
} // namespace FieldConstants

namespace ReefAssist {
inline auto IsRed = []() -> bool {
  return frc::DriverStation::GetAlliance() ==
         frc::DriverStation::Alliance::kRed;
};

inline frc::Pose2d GetReefPose(FieldConstants::Reef reef) {
  return IsRed() ? FieldConstants::Red::kReefPoses[reef]
                 : FieldConstants::Blue::kReefPoses[reef];
};
inline frc::Pose2d getScoringPose(FieldConstants::Reef reef,
                                  FieldConstants::Side side) {

  return ReefAssist::GetReefPose(reef)
      .TransformBy(FieldConstants::kSideTransforms[side])
      .TransformBy(FieldConstants::kEndEffectorTransform.Inverse());
};

inline frc::Pose2d getNearestScoringPose(const frc::Pose2d &robotPose,
                                         FieldConstants::Side side) {
  return robotPose.Nearest({getScoringPose(FieldConstants::Reef::A, side),
                            getScoringPose(FieldConstants::Reef::B, side),
                            getScoringPose(FieldConstants::Reef::C, side),
                            getScoringPose(FieldConstants::Reef::D, side),
                            getScoringPose(FieldConstants::Reef::E, side),
                            getScoringPose(FieldConstants::Reef::F, side)});
}

inline frc::Pose2d getNearestScoringPose(const frc::Pose2d &robotPose) {
  return robotPose.Nearest(
      {getNearestScoringPose(robotPose, FieldConstants::Side::LEFT),
       getNearestScoringPose(robotPose, FieldConstants::Side::RIGHT)});
}

inline frc::Pose2d GetNearestCoralStationPose(const frc::Pose2d &robotPose) {
  if (ReefAssist::IsRed())
    return robotPose
        .Nearest({FieldConstants::Red::kRightCoralStationPose,
                  FieldConstants::Red::kLeftCoralStationPose})
        .TransformBy(FieldConstants::kIntakeTransform);
  else
    return robotPose
        .Nearest({FieldConstants::Blue::kRightCoralStationPose,
                  FieldConstants::Blue::kLeftCoralStationPose})
        .TransformBy(FieldConstants::kIntakeTransform);
}
}; // namespace ReefAssist
#pragma once

#include <optional>

#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonTargetSortMode.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <Eigen/Core>
#include <math.h>
#include <wpi/array.h>

#include <memory>

namespace VisionConstants {

constexpr std::string_view kPhotonCameraName =
    "Arducam_OV2310_USB_Camera"; // Note, we need an in-built pipeline
                                 // changer, probably between auton and
                                 // teleop
const frc::Transform3d kCameraToRobot{
    {13.0_in, 0.5_in, 7.5_in},
    frc::Rotation3d{// transform3d can be constructed with a variety of
                    // variables, so this should be fine
                    0_deg, 0_deg,
                    0_deg}}; // The camera location relative to the robot's
                             // center. Need to change for actual robot

/**A Transform3d that defines the Intake camera offset from the zero (center of
 * robot, between all 4 swerve modules)*/

inline const frc::AprilTagFieldLayout kTagLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};
inline const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{0.2, 0.2, 1};
inline const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.1, 0.1, 0.5};
inline const Eigen::Matrix<double, 3, 1> kFailedTagStdDevs{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
    std::numeric_limits<double>::max()};
} // namespace VisionConstants

class VisionSim; // forward declaration

class Vision : public frc2::SubsystemBase {

public:
  Vision(
      std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
          addVisionMeasurement,
      std::function<frc::Pose2d()> getRobotPose,
      const Eigen::Matrix<double, 3, 1> &initialStdDevs,
      std::function<frc::Pose2d()> getSimulatedPose);
  ~Vision();

  void Periodic() override;
  void SimulationPeriodic() override;
  void UpdateDashboard();

  void GetBestPose();

  bool HasTargets();

  /**
   * Calculate the robot pose estimate using the latest result from a camera.
   *
   * @param estimator The PhotonVision pose estimator that contains the camera
   * and estimated pose.
   * @param lastEstTimestamp The timestamp of the last pose estimated by the
   * given pose estimator.
   *
   * @return A std::optional containing the robot's estimated pose.
   */
  std::optional<photon::EstimatedRobotPose>
  CalculateRobotPoseEstimate(photon::PhotonPoseEstimator &estimator,
                             photon::PhotonCamera &camera,
                             units::second_t &lastEstTimestamp);
  /**Gets the standard deviation of the pose returned by
   * CalculateRobotPoseEstimate*/
  Eigen::Matrix<double, 3, 1>
  GetEstimationStdDevs(frc::Pose2d estimatedPose,
                       photon::PhotonPoseEstimator &estimator,
                       photon::PhotonCamera &camera);
  // ...
public:
  bool IsPoseWithinStdDevs(const frc::Pose2d &incomingPose);
  // ...

private:
  photon::PhotonCamera m_Camera{VisionConstants::kPhotonCameraName};
  photon::PhotonPoseEstimator m_Estimator;

  std::optional<photon::EstimatedRobotPose> m_ApriltagEstimate{std::nullopt};
  // explicit PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTags,
  //                          PoseStrategy strategy, PhotonCamera&& camera,
  //                          frc::Transform3d robotToCamera);
  Eigen::Matrix<double, 3, 1> m_estimatedStdDevs;
  units::time::second_t lastEstTimestamp;
  std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
      m_addVisionMeasurement;

  std::function<frc::Pose2d()> m_referencePose;
  frc::Field2d *m_field_viz;
  std::vector<photon::PhotonPipelineResult> m_resultsVector;

private:
  friend class VisionSim;
  std::unique_ptr<VisionSim> m_sim_state;
};

#include "subsystems/Drivetrain.h"
#include "subsystems/Vision.h"

#include <frc/DataLogManager.h>
#include <photon/simulation/SimCameraProperties.h>
#include <photon/simulation/VisionSystemSim.h>

#include <frc/RobotBase.h>

class VisionSim {
public:
  VisionSim(Vision &vision, std::function<frc::Pose2d()> getSimulatedPose);

  std::function<frc::Pose2d()> m_simulatedPose;
  photon::VisionSystemSim m_vision_sim;
  photon::PhotonCameraSim m_intake_cam_sim;
};

// top do --> add the code from photonvision example to both the cpp and h files
// copy std deviation formula, and potentially make a system where the code
// resets to a set pose if no fiducials are found
//   or I can have it reset to odometry if no fiducials are found
// make code for robot to decide based off of if fidicual id's fall into
// standard deviation, which also means making standard deviation stricter than
// that in code
// Take in code as a Pose3d and convert to Pose2d for drivetrain and other
// estimation, as robot is only moving in 2d, (climb doesn't count") Fix the
// code so it understand difference between old pose and new pose dependent on
// the timestamp

Vision::Vision(
    std::function<void(frc::Pose2d, units::second_t, wpi::array<double, 3U>)>
        addVisionMeasurement,
    std::function<frc::Pose2d()> getRobotPose,
    const Eigen::Matrix<double, 3, 1> &initialStdDevs,
    std::function<frc::Pose2d()> getSimulatedPose)
    : m_Estimator(frc::AprilTagFieldLayout::LoadField(
                      frc::AprilTagField::kDefaultField),
                  photon::CLOSEST_TO_REFERENCE_POSE,
                  VisionConstants::kCameraToRobot),
      m_referencePose(getRobotPose) {
  if constexpr (false) {
    m_sim_state.reset(new VisionSim(*this, std::move(getSimulatedPose)));
    m_field_viz = &m_sim_state->m_vision_sim.GetDebugField();
  } else {
    // uh ohs, memory leak! (its a robot not a rack mounted server)
    m_field_viz = new frc::Field2d{};
    frc::SmartDashboard::PutData("Vision/field", m_field_viz);
  }
  // Inside the constructor body, you can perform additional operations if
  // needed
  m_addVisionMeasurement =
      addVisionMeasurement; // Call the addVisionMeasurement function
  m_Estimator.SetMultiTagFallbackStrategy(
      photon::PoseStrategy::CLOSEST_TO_REFERENCE_POSE);

  frc::DataLogManager::Log("finished initializing vision.");
}

Vision::~Vision() {}

bool Vision::HasTargets() {
  auto Results = m_resultsVector;

  for (auto &res : Results) {
    if (res.HasTargets())
      return true;
  }

  return false;
}

std::optional<photon::EstimatedRobotPose>
Vision::CalculateRobotPoseEstimate(photon::PhotonPoseEstimator &estimator,
                                   photon::PhotonCamera &camera,
                                   units::second_t &lastEstTimestamp) {
  estimator.SetReferencePose(frc::Pose3d{m_referencePose()});

  auto results = m_resultsVector;

  std::optional<photon::EstimatedRobotPose> update = std::nullopt;
  auto timestampDiff = 0_s;

  for (auto &res : results) {
    auto tempUpdate = estimator.Update(res);
    if (tempUpdate.has_value()) {
      update = tempUpdate;
      // std::cout << "it's got an update \n";
    }
    timestampDiff = res.GetLatency();
  }

  units::second_t latestTimestamp =
      frc::Timer::GetFPGATimestamp() - timestampDiff;

  bool newResult =
      units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-7_s;
  if (newResult) {
    lastEstTimestamp = latestTimestamp;
  }

  return update;
}

Eigen::Matrix<double, 3, 1>
Vision::GetEstimationStdDevs(frc::Pose2d estimatedPose,
                             photon::PhotonPoseEstimator &estimator,
                             photon::PhotonCamera &camera) {

  Eigen::Matrix<double, 3, 1> estStdDevs = VisionConstants::kSingleTagStdDevs;
  photon::PhotonPipelineResult latestResult =
      m_resultsVector.back(); // Add declaration for GetLatestResult function

  int numTags = 0; // Declare the variable "numTags" and initialize it to 0

  if (!latestResult.HasTargets()) {
    return VisionConstants::kFailedTagStdDevs;
  }

  // std::cout << "getting stddevs\n";

  auto targets = latestResult.GetTargets();
  auto avgDist = 0.0_m; // Declare and initialize the variable "avgDist"
  auto minDist = 3_m;

  for (const auto &tgt : targets) {
    auto tagPose = estimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
    if (tagPose.has_value()) {
      auto [tag_x, tag_y] = tgt.GetDetectedCorners()[0];
      if ((tag_x > 100 && tag_x < 1400) && (tag_y > 200 && tag_y < 800)) {
        numTags++;
        auto dist = tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
        avgDist += dist;
        minDist = dist < minDist ? dist : minDist;
      }
    }
  }
  if (numTags == 0) {
    return VisionConstants::kFailedTagStdDevs;
  }
  avgDist /= numTags;
  if (numTags > 1) {
    estStdDevs = VisionConstants::kMultiTagStdDevs;
  }
  if (minDist > 8_m) {
    estStdDevs =
        (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
         std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
            .finished();
  } else {
    estStdDevs = estStdDevs * (1 + (minDist.value() * minDist.value() / 2));
  }

  frc::SmartDashboard::PutNumber("Vision/average vision distance",
                                 avgDist.value());
  frc::SmartDashboard::PutNumber("Vision/min vision distance", minDist.value());
  frc::SmartDashboard::PutNumber("Vision/num tags", numTags);
  return estStdDevs;
}

void Vision::Periodic() {
  m_resultsVector = m_Camera.GetAllUnreadResults();

  m_ApriltagEstimate =
      CalculateRobotPoseEstimate(m_Estimator, m_Camera, lastEstTimestamp);

  if (m_ApriltagEstimate.has_value()) {
    auto EstPose2d = m_ApriltagEstimate.value().estimatedPose.ToPose2d();
    auto StdDev = GetEstimationStdDevs(EstPose2d, m_Estimator, m_Camera);
    wpi::array<double, 3U> StdDevArray{StdDev[0], StdDev[1], StdDev[2]};
    m_addVisionMeasurement(EstPose2d, lastEstTimestamp, StdDevArray);
  }

  UpdateDashboard();
}

// void Vision::UpdateDashboard() {
//   m_field_viz->GetObject("Fused Pose")->SetPose(m_referencePose());

//   if (m_intakeApriltagEstimate) {
//     auto robot_pose = m_intakeApriltagEstimate.value().estimatedPose;
//     m_field_viz->GetObject("Intake Cam
//     Pose")->SetPose(robot_pose.ToPose2d());

//     std::vector<frc::Pose2d> reprojected_tags;
//     for (const auto &tag :
//          m_intakeEstimator.GetCamera()->GetLatestResult().GetTargets()) {
//       auto tag_pose =
//           robot_pose.TransformBy(VisionConstants::kIntakeCameraToRobot)
//               .TransformBy(tag.GetBestCameraToTarget());
//       reprojected_tags.push_back(tag_pose.ToPose2d());
//     }

//     m_field_viz->GetObject("Intake Reprojected Tags")
//         ->SetPoses(reprojected_tags);
//   }

//   if (m_shooterApriltagEstimate) {
//     auto robot_pose = m_shooterApriltagEstimate.value().estimatedPose;
//     m_field_viz->GetObject("Shooter Cam
//     Pose")->SetPose(robot_pose.ToPose2d());

//     std::vector<frc::Pose2d> reprojected_tags;
//     for (const auto &tag :
//          m_shooterEstimator.GetCamera()->GetLatestResult().GetTargets()) {
//       auto tag_pose =
//           robot_pose.TransformBy(VisionConstants::kShooterCameraToRobot)
//               .TransformBy(tag.GetBestCameraToTarget());
//       reprojected_tags.push_back(tag_pose.ToPose2d());
//     }

//     m_field_viz->GetObject("Shooter Reprojected Tags")
//         ->SetPoses(reprojected_tags);
//   }

//   UpdateDashboard();
// }

void Vision::UpdateDashboard() {
  m_field_viz->GetObject("Fused Pose")->SetPose(m_referencePose());

  if (m_ApriltagEstimate.has_value()) {
    auto robot_pose = m_ApriltagEstimate.value().estimatedPose;
    m_field_viz->GetObject("Intake Cam Pose")->SetPose(robot_pose.ToPose2d());

    std::vector<frc::Pose2d> reprojected_tags;

    for (auto &res : m_resultsVector) {
      for (const auto &tag : res.GetTargets()) {
        auto tag_pose = robot_pose.TransformBy(VisionConstants::kCameraToRobot)
                            .TransformBy(tag.GetBestCameraToTarget());
        reprojected_tags.push_back(tag_pose.ToPose2d());
      }
    }

    m_field_viz->GetObject("Reprojected Tags")->SetPoses(reprojected_tags);
  }
}

/*****************************SIMULATION*****************************/

photon::SimCameraProperties getShooterCameraProperties() {
  photon::SimCameraProperties ret;
  ret.SetCalibration(1600, 1200, 95_deg);
  ret.SetCalibError(0.15, 0.04);
  ret.SetFPS(24_Hz);
  ret.SetAvgLatency(0.04_s);
  ret.SetLatencyStdDev(0.01_s);

  return ret;
}
photon::SimCameraProperties getIntakeCameraProperties() {
  photon::SimCameraProperties ret;
  ret.SetCalibration(1600, 1200, 95_deg);
  ret.SetCalibError(0.15, 0.04);
  ret.SetFPS(24_Hz);
  ret.SetAvgLatency(0.04_s);
  ret.SetLatencyStdDev(0.01_s);

  return ret;
}

VisionSim::VisionSim(Vision &vision,
                     std::function<frc::Pose2d()> getSimulatedPose)
    : m_simulatedPose(std::move(getSimulatedPose)),
      m_vision_sim("april_tag_sim"),
      m_intake_cam_sim(&vision.m_Camera, getIntakeCameraProperties()) {
  m_vision_sim.AddAprilTags(
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField));
  m_vision_sim.AddCamera(&m_intake_cam_sim, VisionConstants::kCameraToRobot);

  m_intake_cam_sim.EnableDrawWireframe(true);
  m_intake_cam_sim.EnabledProcessedStream(true);
  m_intake_cam_sim.EnableRawStream(true);
  m_intake_cam_sim.SetMaxSightRange(6_m);

  frc::SmartDashboard::PutData("Vision/simulated apriltags",
                               &m_vision_sim.GetDebugField());
}

void Vision::SimulationPeriodic() {
  if (!m_sim_state)
    return;

  m_sim_state->m_vision_sim.Update(m_sim_state->m_simulatedPose());
}
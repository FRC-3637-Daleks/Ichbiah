#include "subsystems/Vision.h"
#include "subsystems/Drivetrain.h"

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
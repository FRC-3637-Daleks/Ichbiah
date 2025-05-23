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
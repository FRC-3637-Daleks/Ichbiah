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

#include <map>
#include <memory>
#include <string>

namespace VisionConstants {

constexpr std::string_view kPhotonCameraName =
    "Arducam_OV2310_USB_Camera"; // Note, we need an in-built pipeline
                                 // changer, probably between auton and
                                 // teleop
const frc::Transform3d kCameraToRobot{
    {13.0_in, -0.5_in, 7.5_in},
    frc::Rotation3d{// transform3d can be constructed with a variety of
                    // variables, so this should be fine
                    0_deg, 0_deg,
                    0_deg}}; // The camera location relative to the robot's
                             // center. Need to change for actual robot

const frc::Transform3d kCameraToEndEffector{
    {-13.5_in, 1_in, 22.5_in}, // assumeing coordinate plan from camera
    frc::Rotation3d{           // in X, Y, Z coordinates
                    // Z = 22.5 because scared to change from Cam2Robot
                    0_deg, 0_deg, 0_deg}};

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

  // Relative stuff
  frc::Transform3d getAprilTagPos(int &tagIDReference);
  frc::Transform3d
  transformCameraToEndEffector(frc::Transform3d CameraRelativePos);
  frc::Transform3d getOffset2NearestReef(frc::Transform3d relativeRobotPos,
                                         int tagID);

  photon::PhotonPipelineResult latestResultStorage;

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

  std::map<std::string, double> tagMap = {
      {"6R", 6.0},   {"6L", -6.0},  {"7R", 6.0},   {"7L", -6.0},  {"8R", 6.0},
      {"8L", -6.0},  {"9R", 6.0},   {"9L", -6.0},  {"10R", 6.0},  {"10L", -6.0},
      {"11R", 6.0},  {"11L", -6.0}, {"17R", 6.0},  {"17L", -6.0}, {"18R", 6.0},
      {"18L", -6.0}, {"19R", 6.0},  {"19L", -6.0}, {"20R", 6.0},  {"20L", -6.0},
      {"21R", 6.0},  {"21L", -6.0}, {"22R", 6.0},  {"22L", -6.0}};

private:
  friend class VisionSim;
  std::unique_ptr<VisionSim> m_sim_state;
};
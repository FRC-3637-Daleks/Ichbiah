#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <optional>

class ROSBridge {
public:
  ROSBridge();

public:
  void PubOdom(const frc::Pose2d &pose, const frc::ChassisSpeeds &vel,
               units::second_t timestamp);

  // publishes the ground truth pose in sim for external use
  void PubSim(const frc::Pose2d &pose);

  void CheckFMS();
  std::optional<frc::Transform2d> GetMapToOdom();

private:
  nt::NetworkTableInstance m_ntInst;

  nt::IntegerPublisher m_pubOdomTimestamp;
  nt::DoubleArrayPublisher m_pubOdomPosLinear;
  nt::DoubleArrayPublisher m_pubOdomPosAngular;
  nt::DoubleArrayPublisher m_pubOdomVelLinear;
  nt::DoubleArrayPublisher m_pubOdomVelAngular;
  nt::DoubleArrayPublisher m_pubOdomAccLinear;

  nt::DoubleArraySubscriber m_subMapToOdomLinear;
  nt::DoubleArraySubscriber m_subMapToOdomAngular;

  nt::IntegerPublisher m_pubSimTimestamp;
  nt::DoubleArrayPublisher m_pubSimPosLinear;
  nt::DoubleArrayPublisher m_pubSimPosAngular;

  std::shared_ptr<nt::NetworkTable> m_fmsTable;
};

#include "subsystems/ROSBridge.h"

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <frc/DriverStation.h>

#include <hal/DriverStation.h>

#include <bit>

ROSBridge::ROSBridge() {
  m_ntInst = nt::NetworkTableInstance::NetworkTableInstance::GetDefault();

  m_ntInst.StartClient4("RosDrivetrain");

  m_pubOdomTimestamp =
      m_ntInst.GetIntegerTopic("/Drivetrain/nt2ros/odom/timestamp").Publish();
  m_pubOdomPosLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/linear")
          .Publish();
  m_pubOdomPosAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/position/angular")
          .Publish();
  m_pubOdomVelLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/linear")
          .Publish();
  m_pubOdomVelAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/velocity/angular")
          .Publish();
  m_pubOdomAccLinear =
      m_ntInst
          .GetDoubleArrayTopic("/Drivetrain/nt2ros/odom/acceleration/linear")
          .Publish();

  m_subMapToOdomLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/ros2nt/map2odom/linear")
          .Subscribe(wpi::array{0., 0., 0.});
  m_subMapToOdomAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/ros2nt/map2odom/angular")
          .Subscribe(wpi::array{0., 0., 0.});

  m_pubSimTimestamp =
      m_ntInst.GetIntegerTopic("/Drivetrain/nt2ros/sim/timestamp").Publish();

  m_pubSimPosLinear =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/sim/position/linear")
          .Publish();

  m_pubSimPosAngular =
      m_ntInst.GetDoubleArrayTopic("/Drivetrain/nt2ros/sim/position/angular")
          .Publish();

  m_fmsTable = m_ntInst.GetTable("FMSInfo");
}

void ROSBridge::CheckFMS() {
  using DS = frc::DriverStation;
  m_fmsTable->PutString("EventName", DS::GetEventName());
  m_fmsTable->PutString("GameSpecificMessage", DS::GetGameSpecificMessage());

  // This is the worst formatting I've ever seen and I can't fix it
  m_fmsTable->PutValue("StationNumber", nt::NetworkTableValue::MakeInteger(
                                            DS::GetLocation().value_or(0)));
  m_fmsTable->PutValue("MatchType",
                       nt::NetworkTableValue::MakeInteger(DS::GetMatchType()));
  m_fmsTable->PutValue(
      "MatchNumber", nt::NetworkTableValue::MakeInteger(DS::GetMatchNumber()));
  m_fmsTable->PutValue("ReplayNumber", nt::NetworkTableValue::MakeInteger(
                                           DS::GetReplayNumber()));

  HAL_ControlWord fms_state;
  HAL_GetControlWord(&fms_state);
  m_fmsTable->PutValue(
      "FMSControlData",
      nt::NetworkTableValue::MakeInteger(std::bit_cast<int32_t>(fms_state)));

  m_fmsTable->PutBoolean("IsRedAlliance",
                         DS::GetAlliance() == DS::Alliance::kRed);
}

void ROSBridge::PubOdom(const frc::Pose2d &pose, const frc::ChassisSpeeds &vel,
                        const units::second_t timestamp) {
  auto current_time_micros = units::microsecond_t{timestamp}.value();
  m_pubOdomTimestamp.Set(current_time_micros, current_time_micros);

  double pubPosLinear[3] = {units::meter_t{pose.X()}.value(),
                            units::meter_t{pose.Y()}.value(), 0};
  m_pubOdomPosLinear.Set(pubPosLinear, current_time_micros);

  double pubPosAngular[3] = {0, 0, pose.Rotation().Radians().value()};
  m_pubOdomPosAngular.Set(pubPosAngular, current_time_micros);

  double pubVelLinear[3] = {units::meters_per_second_t{vel.vx}.value(),
                            units::meters_per_second_t{vel.vy}.value(), 0};
  m_pubOdomVelLinear.Set(pubVelLinear, current_time_micros);

  double pubVelAngular[3] = {0, 0,
                             units::radians_per_second_t{vel.omega}.value()};
  m_pubOdomVelAngular.Set(pubVelAngular, current_time_micros);

  m_ntInst.Flush();
}

void ROSBridge::PubSim(const frc::Pose2d &pose) {
  const auto current_time_micros = nt::Now();

  double pubPosLinear[3] = {units::meter_t{pose.X()}.value(),
                            units::meter_t{pose.Y()}.value(), 0};
  m_pubSimPosLinear.Set(pubPosLinear, current_time_micros);

  double pubPosAngular[3] = {0, 0, pose.Rotation().Radians().value()};
  m_pubSimPosAngular.Set(pubPosAngular, current_time_micros);

  m_pubSimTimestamp.Set(current_time_micros, current_time_micros);

  m_ntInst.Flush();
}

std::optional<frc::Transform2d> ROSBridge::GetMapToOdom() {
  if (!m_subMapToOdomAngular.Exists() || !m_subMapToOdomLinear.Exists())
    return std::nullopt;

  // disgusting allocations, can be optimized
  auto orientation = m_subMapToOdomAngular.Get();
  auto offset = m_subMapToOdomLinear.Get();
  if (orientation.size() < 3 || offset.size() < 3)
    return std::nullopt;

  const units::meter_t x{offset[0]};
  const units::meter_t y{offset[1]};
  const units::radian_t theta{orientation[2]};

  return frc::Transform2d{x, y, theta};
}
#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

// Only place constants here needed by other components
namespace KrakenModuleConstants {

// website value
constexpr auto kPhysicalMaxSpeed = 12.9_fps;

} // namespace KrakenModuleConstants

namespace PracticeModuleConstants {

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 15.7_fps;

} // namespace PracticeModuleConstants

// forward declaration
class SwerveModuleSim;

/**
 * The SwerveModule helper class consists of a steer motor and a drive motor
 * (both Falcon 500s/Krakens/Talon FXs).
 * Additionally, there is an absolute encoder (CANCoder) which regardless of
 * start position, reports the exact heading of the wheels.
 * Each module can be commanded to a certain state, that is,
 * its wheel will be driven at the specified velocity in the specified
 * direction. The Drivetrain subsystem makes use of SwerveModule objects so that
 * it doesn't need to deal with directly commanding each motor.
 *
 */
class SwerveModule {
public:
  /**
   * The SignalGroup struct encapsulates all the signals which are used
   * at a high frequency by a SwerveModule. SignalGroup objects can be
   * copied to maintain multiple thread-safe views into the module state.
   */
  struct SignalGroup {
    using position_signal_t =
        ctre::phoenix6::StatusSignal<units::angle::turn_t>;
    using velocity_signal_t = ctre::phoenix6::StatusSignal<
        units::angular_velocity::turns_per_second_t>;

    position_signal_t m_drivePosition;
    velocity_signal_t m_driveVelocity;
    position_signal_t m_steerPosition;
    velocity_signal_t m_steerVelocity;

    // Returns the meters driven based on encoder reading.
    units::meter_t GetModuleDistance();

    // Returns the velocity of the module in m/s.
    units::meters_per_second_t GetModuleVelocity();

    // Returns the module heading in the scope [-180,180] degrees.
    frc::Rotation2d GetModuleHeading();

    // Combines GetModuleDistance() and GetModuleHeading().
    frc::SwerveModulePosition GetPosition();

    // Combines GetModuleVelocity() and GetModuleHeading().
    frc::SwerveModuleState GetState();

    // Efficiently refreshes a list of signal groups together
    template <std::same_as<SignalGroup>... T>
    static void RefreshAllSignals(T &...groups) {
      ctre::phoenix6::BaseStatusSignal::RefreshAll(
          groups.m_drivePosition..., groups.m_driveVelocity...,
          groups.m_steerPosition..., groups.m_steerVelocity...);
    }

    // Similar to RefreshAllSignals but instead waits until all signals
    // receives a fresh value from the CAN bus.
    // Blocks until specified timeout. Returns true when timeout exceeded
    // or other error
    template <std::same_as<SignalGroup>... T>
    static bool WaitForAllSignals(units::time::second_t timeout, T &...groups) {
      return ctre::phoenix6::BaseStatusSignal::WaitForAll(
                 timeout, groups.m_drivePosition..., groups.m_driveVelocity...,
                 groups.m_steerPosition...,
                 groups.m_steerVelocity...) != ctre::phoenix::StatusCode::OK;
    }

    template <auto N>
    static void RefreshAllSignals(std::array<SignalGroup, N> &groups) {
      std::apply(
          [](auto &&...gs) {
            RefreshAllSignals(std::forward<decltype(gs)>(gs)...);
          },
          groups);
    }

    template <auto N>
    static bool WaitForAllSignals(units::time::second_t timeout,
                                  std::array<SignalGroup, N> &groups) {
      return std::apply(
          [timeout](auto &&...gs) {
            return WaitForAllSignals(timeout,
                                     std::forward<decltype(gs)>(gs)...);
          },
          groups);
    }
  };

public:
  // The ctor of the SwerveModule class.
  SwerveModule(const std::string name, const int driveMotorId,
               const int steerMotorId, const int absoluteEncoderId);

  // Need to define destructor to make simulation code compile
  ~SwerveModule();

  // IMPORTANT: Need to refresh signals once per loop.
  // Getters will not return different values until signals are refreshed again
  void RefreshSignals();

  SignalGroup GetSignals() { return m_signals; }

  // This one is even more efficient than RefreshSignals as it groups ALL
  // swerve module signals into a single call
  template <std::same_as<SwerveModule>... T>
  static void RefreshAllSignals(T &...modules) {
    SignalGroup::RefreshAllSignals(modules.m_signals...);
  }
  template <auto N>
  static void RefreshAllSignals(std::array<SwerveModule, N> &modules) {
    std::apply(
        [](auto &&...ms) {
          RefreshAllSignals(std::forward<decltype(ms)>(ms)...);
        },
        modules);
  }

  // Returns the meters driven based on encoder reading.
  units::meter_t GetModuleDistance() { return m_signals.GetModuleDistance(); }

  // Returns the velocity of the module in m/s.
  units::meters_per_second_t GetModuleVelocity() {
    return m_signals.GetModuleVelocity();
  }

  // Returns the module heading in the scope [-180,180] degrees.
  frc::Rotation2d GetModuleHeading() { return m_signals.GetModuleHeading(); }

  // Combines GetModuleDistance() and GetModuleHeading().
  frc::SwerveModulePosition GetPosition() { return m_signals.GetPosition(); }

  // Combines GetModuleVelocity() and GetModuleHeading().
  frc::SwerveModuleState GetState() { return m_signals.GetState(); }

  const std::string &GetName() { return m_name; }

  void CoastMode(bool coast);

  void SetEncoderOffset();

  void ZeroAbsEncoders();

  void SyncEncoders();

  // Commands the module to accelerate to a certain velocity and take on a
  // certain heading.
  void SetDesiredState(const frc::SwerveModuleState &state);

  // Sends the current swerve module state to the SmartDashboard.
  void UpdateDashboard();

  // Run physics simulation and update the hardware
  void SimulationPeriodic();

private:
  // Returns the absolute position of the steer motor in radians
  units::radian_t GetAbsoluteEncoderPosition();

  const std::string m_name; // Useful to identify the module.

  ctre::phoenix6::hardware::TalonFX m_driveMotor;

  ctre::phoenix6::hardware::TalonFX m_steerMotor;

  // Keeps track of the module heading between power cycles.
  ctre::phoenix6::hardware::CANcoder m_absoluteEncoder;

private: // signal object to cache
  SignalGroup m_signals;

private:
  friend class SwerveModuleSim;
  std::unique_ptr<SwerveModuleSim> m_sim_state;
};

#include "swerve/SwerveModule.h"

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/filter/LinearFilter.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <iostream>
#include <numbers>
#include <random>

namespace KrakenModuleConstants {
// Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 70_A;
constexpr auto kSteerMotorCurrentLimit = 30_A;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

// Indicates time from neutral to full output
constexpr auto kRampRate = 0.2_s;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 5.36;     // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution = // Linear distance per
                                                    // revolution of motor
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment =
    .0101_kg_sq_m;                     // calculated based on a weight of 70lbs
constexpr auto kMotorSpeed = 5800_rpm; // Website value
constexpr auto kDriveMaxAcceleration = 500_tr_per_s_sq;
constexpr auto kDriveTargetAcceleration = 300_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 12.8;
constexpr auto kSteerMoment = 0.005_kg_sq_m;
constexpr auto kSteerAcceleration =
    135.7_tr_per_s_sq * 2; // Measured empirically, rough guess
constexpr auto kSteerSpeed = kMotorSpeed / kSteerGearReduction;

constexpr double kDriveP = 0.02, kDriveI = 0.000, kDriveD = 0.001;
constexpr double kSteerP = 10, kSteerI = 0, kSteerD = 0.022, kSteerS = 0.03;

const auto MotorModel = [](int N = 1) { return frc::DCMotor::KrakenX60FOC(N); };

} // namespace KrakenModuleConstants

namespace PracticeModuleConstants {
// Motor outputs under 4% will just be cut to 0 (brake)
constexpr double kNeutralDeadband = 0.04;

// Current Limit configs
constexpr auto kDriveMotorCurrentLimit = 60_A;
constexpr auto kSteerMotorCurrentLimit = 30_A;
// Can exceed limit for 40ms seconds
constexpr auto kCurrentLimitPeriod = 40_ms;

// Indicates time from neutral to full output
constexpr auto kRampRate = 0.2_s;

constexpr auto kWheelDiameter = 4_in;

constexpr double kDriveEncoderReduction = 6.75;     // reduction in drive motor
constexpr auto kDriveEncoderDistancePerRevolution = // Linear distance per
                                                    // revolution of motor
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;
constexpr auto kWheelMoment =
    .0101_kg_sq_m; // calculated based on a weight of 70lbs
constexpr auto kMotorSpeedChoreo = 5104_rpm; // choreo value
constexpr auto kMotorSpeed = 6080_rpm;       // Website value
constexpr auto kDriveMaxAcceleration = 500_tr_per_s_sq;
constexpr auto kDriveTargetAcceleration = 300_tr_per_s_sq;
constexpr auto kDistanceToRotations = kDriveEncoderDistancePerRevolution / 1_tr;

constexpr double kSteerGearReduction = 150.0 / 7.0;
constexpr auto kSteerMoment =
    0.0001_kg_sq_m; // Reduced to near 0-mass for smooth sim driving
constexpr auto kSteerAcceleration =
    135.7_tr_per_s_sq * 2; // Measured empirically, rough guess
constexpr auto kSteerSpeed = kMotorSpeed / kSteerGearReduction;

constexpr double kDriveP = 0, kDriveI = 0.1, kDriveD = 0;
constexpr double kSteerP = 10, kSteerI = 0, kSteerD = 0.02, kSteerS = 0.03;

const auto MotorModel = [](int N = 1) { return frc::DCMotor::Falcon500FOC(N); };

} // namespace PracticeModuleConstants

using namespace KrakenModuleConstants;
class SwerveModuleSim {
public:
  SwerveModuleSim(SwerveModule &module)
      : m_driveSim(std::move(module.m_driveMotor.GetSimState())),
        m_steerSim(std::move(module.m_steerMotor.GetSimState())),
        m_encoderSim(std::move(module.m_absoluteEncoder.GetSimState())),
        m_wheelModel(frc::LinearSystemId::DCMotorSystem(
                         MotorModel(), kWheelMoment, kDriveEncoderReduction),
                     MotorModel()),
        m_swivelModel(frc::LinearSystemId::DCMotorSystem(
                          MotorModel(), kSteerMoment, kSteerGearReduction),
                      MotorModel()) {
    static std::random_device rng;
    std::uniform_real_distribution dist(-0.5, 0.5);

    // randomize starting positions
    m_swivelModel.SetState(dist(rng) * 1_tr, 0_rpm);
  }

  void update();

private:
  // hooks to hardware abstraction layer

  ctre::phoenix6::sim::TalonFXSimState m_driveSim, m_steerSim;
  ctre::phoenix6::sim::CANcoderSimState m_encoderSim;

  // tracks the simulation state for each wheel
  frc::sim::DCMotorSim m_wheelModel, m_swivelModel;
};

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId, const int absoluteEncoderId)
    : m_name{name}, m_driveMotor(driveMotorId, "Drivebase"),
      m_steerMotor(steerMotorId, "Drivebase"),
      m_absoluteEncoder(absoluteEncoderId, "Drivebase"),
      m_signals{m_driveMotor.GetPosition(), m_driveMotor.GetVelocity(),
                m_steerMotor.GetPosition(), //< FusedCANCoder
                m_steerMotor.GetVelocity()},
      m_sim_state(new SwerveModuleSim(*this)) {

  // Reduce clutter in this function
  using namespace ctre::phoenix6;
  using namespace units;

  configs::TalonFXConfiguration steerConfig, driveConfig;

  steerConfig.WithMotorOutput(
      configs::MotorOutputConfigs{}
          .WithNeutralMode(signals::NeutralModeValue::Brake)
          .WithInverted(true));

  driveConfig.WithMotorOutput(
      configs::MotorOutputConfigs{}
          .WithNeutralMode(signals::NeutralModeValue::Brake)
          .WithDutyCycleNeutralDeadband(kNeutralDeadband));

  driveConfig.WithOpenLoopRamps(configs::OpenLoopRampsConfigs{}
                                    .WithDutyCycleOpenLoopRampPeriod(kRampRate)
                                    .WithVoltageOpenLoopRampPeriod(kRampRate)
                                    .WithTorqueOpenLoopRampPeriod(kRampRate));

  driveConfig.WithClosedLoopRamps(
      configs::ClosedLoopRampsConfigs{}
          .WithDutyCycleClosedLoopRampPeriod(kRampRate)
          .WithVoltageClosedLoopRampPeriod(kRampRate)
          .WithTorqueClosedLoopRampPeriod(kRampRate));

  // CTRE Alleges that the new firmware has sensible default current limits
  // driveConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
  //   .WithSupplyCurrentLimitEnable(true)
  //   .WithSupplyCurrentLimit(kDriveMotorCurrentLimit)
  // );

  // steerConfig.WithCurrentLimits(configs::CurrentLimitsConfigs{}
  //   .WithSupplyCurrentLimitEnable(true)
  //   .WithSupplyCurrentLimit(kSteerMotorCurrentLimit)
  // );

  // max duty cycle / corresponding velocity
  constexpr auto kDriveV = 1.0 / (kPhysicalMaxSpeed / kDistanceToRotations);
  // constexpr auto kDriveA
  // = 1.0/units::turns_per_second_squared_t{kDriveMaxAcceleration};
  driveConfig.WithSlot0(
      configs::Slot0Configs{}
          .WithKP(kDriveP)
          .WithKI(kDriveI)
          .WithKD(kDriveD)
          .WithKV(ctre::unit::scalar_per_turn_per_second_t{kDriveV}.value()));

  constexpr auto kSteerV = 1.0 / units::turns_per_second_t{kSteerSpeed};
  constexpr auto kSteerA =
      1.0 / units::turns_per_second_squared_t{kSteerAcceleration};
  steerConfig.WithSlot0(configs::Slot0Configs{}
                            .WithKP(kSteerP)
                            .WithKI(kSteerI)
                            .WithKD(kSteerD)
                            .WithKV(kSteerV.value())
                            .WithKA(kSteerA.value())
                            .WithKS(kSteerS));

  driveConfig.WithMotionMagic(
      configs::MotionMagicConfigs{}.WithMotionMagicAcceleration(
          kDriveTargetAcceleration));

  steerConfig.WithMotionMagic(
      configs::MotionMagicConfigs{}
          .WithMotionMagicCruiseVelocity(kSteerSpeed)
          .WithMotionMagicAcceleration(kSteerAcceleration)
          .WithMotionMagicExpo_kV(12.0_V * kSteerV)
          .WithMotionMagicExpo_kA(12.0_V * kSteerA));

  // this object has no "With*" API for some reason
  ctre::phoenix6::configs::ClosedLoopGeneralConfigs steerClosedLoopConfig{};
  steerClosedLoopConfig.ContinuousWrap = true;
  steerConfig.WithClosedLoopGeneral(steerClosedLoopConfig);

  steerConfig.WithFeedback(
      configs::FeedbackConfigs{}
          .WithFeedbackSensorSource(
              signals::FeedbackSensorSourceValue::FusedCANcoder)
          .WithFeedbackRemoteSensorID(m_absoluteEncoder.GetDeviceID())
          .WithRotorToSensorRatio(kSteerGearReduction)
          .WithSensorToMechanismRatio(1.0));

  /* Sometimes configuration fails, so we check the return code
   * and retry if needed.
   */
  int retries = 4;
  while (auto ret = m_driveMotor.GetConfigurator().Apply(driveConfig, 500_ms)) {
    if (retries-- == 0) {
      // when ret is non-zero, that means there's an error
      std::cerr << "ERROR Applying Drive Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << driveMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  retries = 4;
  while (auto ret = m_steerMotor.GetConfigurator().Apply(steerConfig, 500_ms)) {
    if (retries-- == 0) {
      std::cerr << "ERROR Applying Steer Motor Configs for " << m_name
                << std::endl;
      std::cerr << "Talon ID: " << steerMotorId << ", Error: " << ret
                << std::endl;
      break;
    }
  }

  frc::DataLogManager::Log(
      fmt::format("Finished initializing {} swerve module", m_name));
}

SwerveModule::~SwerveModule() {}

void SwerveModule::RefreshSignals() {
  /* Refreshes all this modules signals at once.
   * This should improve performance
   */
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
      m_signals.m_drivePosition, m_signals.m_driveVelocity,
      m_signals.m_steerPosition, m_signals.m_steerVelocity);
}

units::meter_t SwerveModule::SignalGroup::GetModuleDistance() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_drivePosition, m_driveVelocity);
  return position * kDistanceToRotations;
}

units::meters_per_second_t SwerveModule::SignalGroup::GetModuleVelocity() {
  return m_driveVelocity.GetValue() * kDistanceToRotations;
}

frc::Rotation2d SwerveModule::SignalGroup::GetModuleHeading() {
  const auto position =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_steerPosition, m_steerVelocity);
  return position.convert<units::degree>();
}

frc::SwerveModulePosition SwerveModule::SignalGroup::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::SignalGroup::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::CoastMode(bool coast) {
  if (coast) {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Coast);
  } else {
    m_steerMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
    m_driveMotor.SetNeutralMode(
        ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
}

void SwerveModule::SetEncoderOffset() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  auto position = m_absoluteEncoder.GetAbsolutePosition().GetValue();
  magConfig.WithMagnetOffset(-position);
  magConfig.WithAbsoluteSensorDiscontinuityPoint(0.5_tr);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);

  SyncEncoders();
}

void SwerveModule::ZeroAbsEncoders() {
  ctre::phoenix6::configs::MagnetSensorConfigs magConfig;
  magConfig.WithMagnetOffset(0_tr);
  magConfig.WithAbsoluteSensorDiscontinuityPoint(0.5_tr);
  magConfig.WithSensorDirection(
      ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive);

  m_absoluteEncoder.GetConfigurator().Apply(magConfig, 50_ms);
}

void SwerveModule::SyncEncoders() {
  m_steerMotor.SetPosition(m_absoluteEncoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  auto state = referenceState;
  state.Optimize(GetModuleHeading());
  state.speed *= (state.angle - GetModuleHeading()).Cos();

  m_driveMotor.SetControl(
      ctre::phoenix6::controls::MotionMagicVelocityDutyCycle{
          state.speed / kDistanceToRotations}
          .WithEnableFOC(true)
          .WithSlot(0));

  m_steerMotor.SetControl(
      ctre::phoenix6::controls::MotionMagicExpoDutyCycle{state.angle.Radians()}
          .WithEnableFOC(true)
          .WithSlot(0));
}

void SwerveModule::UpdateDashboard() {
  const auto state = GetState();

  frc::SmartDashboard::PutNumber(
      fmt::format("Swerve/{}/heading (degrees)", m_name),
      state.angle.Degrees().value());

  frc::SmartDashboard::PutNumber(fmt::format("Swerve/{}/speed (mps)", m_name),
                                 state.speed.convert<units::mps>().value());
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return m_absoluteEncoder.GetAbsolutePosition().GetValue();
}

// Simulation
void SwerveModule::SimulationPeriodic() {
  if (m_sim_state)
    m_sim_state->update();
}

void SwerveModuleSim::update() {
  m_driveSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_steerSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_encoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  // Simulate the wheel swiveling
  m_swivelModel.SetInputVoltage(m_steerSim.GetMotorVoltage());
  m_swivelModel.Update(20_ms);
  // cancoder is on mechanism and is inverted from the falcon's rotor
  m_encoderSim.SetRawPosition(-m_swivelModel.GetAngularPosition());
  m_encoderSim.SetVelocity(-m_swivelModel.GetAngularVelocity());
  m_steerSim.SetRawRotorPosition(m_swivelModel.GetAngularPosition() *
                                 kSteerGearReduction);
  m_steerSim.SetRotorVelocity(m_swivelModel.GetAngularVelocity() *
                              kSteerGearReduction);
  m_steerSim.SetRotorAcceleration(m_swivelModel.GetAngularAcceleration() *
                                  kSteerGearReduction);

  // Simulate the wheel turning (ignoring changes in traction)
  m_wheelModel.SetInputVoltage(m_driveSim.GetMotorVoltage());
  m_wheelModel.Update(20_ms);

  m_driveSim.SetRawRotorPosition(m_wheelModel.GetAngularPosition() *
                                 kDriveEncoderReduction);
  m_driveSim.SetRotorVelocity(m_wheelModel.GetAngularVelocity() *
                              kDriveEncoderReduction);
  m_driveSim.SetRotorAcceleration(m_wheelModel.GetAngularAcceleration() *
                                  kDriveEncoderReduction);
}
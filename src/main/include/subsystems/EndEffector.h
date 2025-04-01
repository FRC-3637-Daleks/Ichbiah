#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkFlex.h>

#include <memory>

class EndEffectorSim;

class EndEffector : public frc2::SubsystemBase {
public:
  EndEffector();
  ~EndEffector();

  void Periodic() override;
  void UpdateDashboard();
  void InitVisualization(frc::MechanismObject2d *elevator_end);
  void UpdateVisualization();

  // -1 <= speed_pct <= 1
  void SetSpeed(double speed_pct);
  void MotorStop() { SetSpeed(0.0); }

  bool IsInnerBreakBeamBroken();
  bool IsOuterBreakBeamBroken();
  bool HasCoral();

  frc2::CommandPtr MotorCommand(double speed_pct);
  frc2::CommandPtr MotorForwardCommand();
  frc2::CommandPtr MotorBackwardCommand();
  frc2::CommandPtr EffectorIn();
  frc2::CommandPtr EffectorContinue();
  frc2::CommandPtr EffectorOut();
  frc2::CommandPtr EffectorOutToL1();
  frc2::CommandPtr Intake();

private:
  rev::spark::SparkFlex m_EndEffectorMotor;
  rev::spark::SparkLimitSwitch &m_InnerBreakBeam;
  rev::spark::SparkLimitSwitch &m_OuterBreakBeam;

  // visualization stuff
private:
  frc::MechanismLigament2d *m_mech_endeffector_base, *m_mech_backbeam,
      *m_mech_frontbeam, *m_mech_spinner;

  // simulation stuff
private:
  friend class EndEffectorSim;
  std::unique_ptr<EndEffectorSim> m_sim_state;

public:
  void SimulationPeriodic() override;
  void SimulateNewCoral();
  void SimulatePreloadCoral();
};
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

  frc2::CommandPtr MotorForward();
  frc2::CommandPtr MotorBack();
  frc2::CommandPtr MotorStop();

  bool isForwardBreakBeamBroken();
  bool isBackwardBreakBeamBroken();
  bool hasCoral();

  frc2::CommandPtr WhileOut();
  frc2::CommandPtr WhileIn();
  frc2::CommandPtr EffectorIn();
  frc2::CommandPtr EffectorContinue();
  frc2::CommandPtr EffectorOut();
  frc2::CommandPtr Intake();

private:
  // frc::DigitalInput m_ForwardBreakBeam;
  // frc::DigitalInput m_BackwardBreakBeam;
  rev::spark::SparkFlex m_EndEffectorMotor;
  rev::spark::SparkLimitSwitch &m_ForwardBreakBeam;
  rev::spark::SparkLimitSwitch &m_BackwardBreakBeam;

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
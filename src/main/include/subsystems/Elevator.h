#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <memory>
#include <numbers>

// Forward Declaration
class ElevatorSim;

class Elevator : public frc2::SubsystemBase {
public:
  Elevator();
  ~Elevator(); // Need for reasons

  void Periodic() override;
  void UpdateDashboard();
  void InitVisualization(frc::MechanismObject2d *elevator_base);
  frc::MechanismLigament2d *GetElevatorLigament() { return m_mech_current; }
  void UpdateVisualization();

  enum Level : int { INTAKE = 0, L1, L2, L3, L4, N };

  // Return bool on if its at the peram: pos
  bool IsAtPos(units::centimeter_t pos);
  bool IsAtLevel(Level level);

  // Sets the motor position (ends as soon as run)
  void SetGoalHeight(units::centimeter_t length);
  void SetGoalHeight(Level level);
  // Moves the motor up and down manually
  void MotorMoveUp();
  void MotorMoveDown();
  void MotorStop();

  // Gets the encoder position (used mostly by other functions)
  units::centimeter_t GetEndEffectorHeight();

  bool isAtTop();
  bool isAtBottom();
  bool shouldHome();

public:
  // Manual Override Commands
  frc2::CommandPtr MoveUp();
  frc2::CommandPtr MoveDown();
  frc2::CommandPtr Hold();

  // Ensures sensor is homed
  frc2::CommandPtr HomeEncoder();

  // Main command, commands elevator to goal and then ends when it reaches that
  // point
  frc2::CommandPtr GoToLevel(Level goal);

private:
  ctre::phoenix6::hardware::TalonFX m_leadMotor;
  ctre::phoenix6::hardware::TalonFX m_followerMotor;

#ifdef ELEVATOR_TOP_LIMIT_SWITCH
  frc::DigitalInput m_forwardLimit;
#endif
  frc::DigitalInput m_reverseLimit;

private: // visualization members
  frc::MechanismLigament2d *m_mech_current, *m_mech_goal;

private: // simulation related members
  friend class ElevatorSim;
  std::unique_ptr<ElevatorSim> m_sim_state;
  void SimulationPeriodic() override;
};
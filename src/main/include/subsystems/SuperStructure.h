#pragma once

#include "subsystems/Elevator.h"
#include "subsystems/EndEffector.h"
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SubsystemBase.h>

// Forward Declaration
class SuperStructureSim;

class SuperStructure : public frc2::SubsystemBase {
public:
  SuperStructure(Elevator &elevator, EndEffector &end_effector);
  ~SuperStructure(); // Need for reasons

  void Periodic() override;
  void UpdateDashboard();
  void InitVisualization(frc::MechanismObject2d *elevator_root);
  void UpdateVisualization();

  Elevator &m_elevator;
  EndEffector &m_endeffector;

  bool baseAtPos;
  frc2::CommandPtr prePlace(Elevator::Level level);
  frc2::CommandPtr Intake();
  frc2::CommandPtr Score();

private:
  friend class SuperStructureSim;
  std::unique_ptr<SuperStructureSim> m_sim_state;
  void SimulationPeriodic() override;
};
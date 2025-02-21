#pragma once

#include "subsystems/Elevator.h"
#include "subsystems/EndEffector.h"
#include <frc2/command/SubsystemBase.h>

// Forward Declaration
class SuperStructureSim;

class SuperStructure : public frc2::SubsystemBase {
public:
  SuperStructure(Elevator& elevator, EndEffector& end_effector);
  ~SuperStructure(); // Need for reasons

  Elevator m_elevator;
  EndEffector m_endeffector;

  frc2::CommandPtr moveElevatorTo(Elevator::Level level);
  bool baseAtPos;
  frc2::CommandPtr prePlace(Elevator::Level level);

private:
  friend class SuperStructureSim;
  std::unique_ptr<SuperStructureSim> m_sim_state;
  void SimulationPeriodic() override;
};
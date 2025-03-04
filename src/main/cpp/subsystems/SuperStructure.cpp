#include "subsystems/SuperStructure.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

namespace SuperstructureConstants {};

class SuperStructureSim {
public:
  SuperStructureSim(SuperStructure &superstructure);
};

SuperStructure::SuperStructure(Elevator &elevator, EndEffector &end_effector)
    : m_elevator(elevator), m_endeffector(end_effector),
      m_sim_state{new SuperStructureSim{*this}} {};

void SuperStructure::Periodic() { UpdateDashboard(); }

void SuperStructure::UpdateDashboard() { UpdateVisualization(); }

void SuperStructure::InitVisualization(frc::MechanismObject2d *elevator_root) {
  m_elevator.InitVisualization(elevator_root);
  m_endeffector.InitVisualization(m_elevator.GetElevatorLigament());
}

void SuperStructure::UpdateVisualization() {}

frc2::CommandPtr SuperStructure::prePlace(Elevator::Level level) {
  return /*rember to implement intake here*/ m_endeffector.EffectorContinue()
      .AlongWith(m_elevator.GoToLevel(level));
};

frc2::CommandPtr SuperStructure::Intake() {
  return m_elevator.GoToLevel(m_elevator.INTAKE)
      .AndThen(m_endeffector.Intake().Until(
          [this]() -> bool { return m_endeffector.hasCoral(); }));
}

// Pre-requisit is having coral && being at the right
frc2::CommandPtr SuperStructure::Score() {
  return m_endeffector.EffectorOut()
      .AndThen(frc2::cmd::Wait(0.25_s))
      .AndThen(m_elevator.GoToLevel(m_elevator.INTAKE));
}

SuperStructure::~SuperStructure() {}

SuperStructureSim::SuperStructureSim(SuperStructure &superstructure) {}

void SuperStructure::SimulationPeriodic() {}

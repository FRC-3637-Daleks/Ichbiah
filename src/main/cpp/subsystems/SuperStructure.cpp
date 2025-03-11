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
  // Always intake first. Intake will instantly exit if we already have coral
  // Parallel empty command means the command runs indefinitely even after
  // completing the components
  return Intake()
      .AndThen(m_elevator.GoToLevel(level).AlongWith(
          m_endeffector.EffectorContinue()))
      .AlongWith(Run([] {})); // keep parallel
};

frc2::CommandPtr SuperStructure::Intake() {
  // The command will run until the endeffector intakes
  return m_endeffector.Intake().DeadlineFor(
      m_elevator.GoToLevel(Elevator::INTAKE));
}

// Pre-requisit is having coral && being at the right
frc2::CommandPtr SuperStructure::Score(Elevator::Level level) {
  auto out_cmd = (level == Elevator::L1) ? m_endeffector.EffectorOutToL1()
                                         : m_endeffector.EffectorOut();

  // Intake first, if we have coral then no problem
  // Go to the level, wait to get there
  // Then output at the speed for said level
  // 2 second timeout just in case pressed with no coral inside
  return Intake()
      .AndThen(m_elevator.GoToLevel(level))
      .AndThen(std::move(out_cmd))
      .WithTimeout(2.0_s); // if left unchecked
}

SuperStructure::~SuperStructure() {}

SuperStructureSim::SuperStructureSim(SuperStructure &superstructure) {}

void SuperStructure::SimulationPeriodic() {}

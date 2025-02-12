#include "subsystems/SuperStructure.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include <iostream>

namespace SuperstructureConstants {

};

class SuperStructureSim {
  public:
    SuperStructureSim(SuperStructure& superstructure);

  public:
    frc::Mechanism2d m_mech{3, 3};

    frc::MechanismRoot2d* m_root = m_mech.GetRoot("climber", 1, 0);

    frc::MechanismLigament2d* m_elevator =
      m_root->Append<frc::MechanismLigament2d>("elevator", 1, 90_deg, 6,
      frc::Color::kBlue);

    frc::MechanismLigament2d* m_wrist =
      m_elevator->Append<frc::MechanismLigament2d>(
          "wrist", 1, 90_deg, 6, frc::Color8Bit{frc::Color::kGreen});
};

SuperStructure::SuperStructure() :
                                    m_sim_state{new SuperStructureSim{*this}}
{
    frc::SmartDashboard::PutData("Mech2d", &m_sim_state->m_mech);
};

frc2::CommandPtr SuperStructure::prePlace(Elevator::Level level) {
    return m_endeffector.EffectorContinue().AlongWith(m_elevator.GoToLevel(level));
};

frc2::CommandPtr SuperStructure::moveElevatorTo(Elevator::Level level) {
    return m_elevator.GoToLevel(level);
};

SuperStructure::~SuperStructure() {}

SuperStructureSim::SuperStructureSim(SuperStructure& superstructure) {
    
}

void SuperStructure::SimulationPeriodic() {
    m_sim_state->m_elevator->SetLength((m_elevator.GetEndEffectorHeight()).value()/50);
}

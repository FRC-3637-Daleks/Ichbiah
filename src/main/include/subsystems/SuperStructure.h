#pragma once


#include <frc2/command/SubsystemBase.h>
#include "subsystems/Elevator.h"

// Forward Declaration
class SuperStructureSim;

class SuperStructure: public frc2::SubsystemBase {
public:
    SuperStructure();
    ~SuperStructure();  // Need for reasons

    Elevator m_elevator;

    frc2::CommandPtr moveElevatorTo(Elevator::Level level);

private:
    friend class SuperStructureSim;
    std::unique_ptr<SuperStructureSim> m_sim_state;
    void SimulationPeriodic() override;
};
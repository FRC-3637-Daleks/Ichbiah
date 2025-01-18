#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

namespace CoralIntakeConstants{
    int breakbeamID = 30;
}

class CoralIntake {
    public:
    
    CoralIntake();
    bool getBreakbeamState();

    private:
    
    frc::DigitalInput m_breakbeam{CoralIntakeConstants::breakbeamID};

};
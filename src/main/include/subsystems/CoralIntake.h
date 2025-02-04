#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

namespace CoralIntakeConstants{
    int kBreakbeamID = 30;
    int kBreakbeam2ID = 40;
    int kBreakbeam3ID = 50;
    int kintakeStateID = 60;
}

class CoralIntake: public frc2::SubsystemBase {
    public:
    
    void Periodic();
    void SetState();

    CoralIntake();
    
    bool getBreakbeamState();
    bool isBreakbeamBroken();
    bool isBreakbeam2Broken();
    bool isBreakbeam3Broken();
    bool getintakeState();
    void SetState(bool state);
    bool GetState();
    
   
    private:

    frc::DigitalInput m_Breakbeam{CoralIntakeConstants::kBreakbeamID};
    frc::DigitalInput m_Breakbeam2{CoralIntakeConstants::kBreakbeam2ID};
    frc::DigitalInput m_Breakbeam3{CoralIntakeConstants::kBreakbeam3ID};
    bool intakeState;
    frc::DigitalInput m_intakeState{CoralIntakeConstants::kintakeStateID};
};
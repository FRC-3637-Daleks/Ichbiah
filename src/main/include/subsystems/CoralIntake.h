#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

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

    frc::DigitalInput m_Breakbeam;
    frc::DigitalInput m_Breakbeam2;
    frc::DigitalInput m_Breakbeam3;
    frc::DigitalInput m_intakeState;
    bool intakeState;

};
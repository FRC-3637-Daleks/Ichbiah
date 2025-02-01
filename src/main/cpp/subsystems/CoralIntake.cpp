#include "subsystems/CoralIntake.h"

CoralIntake::CoralIntake(){
}

bool CoralIntake::getBreakbeamState(){
    return m_Breakbeam.Get();
}

bool CoralIntake::isBreakbeamBroken(){
    return !(m_Breakbeam.Get());
}

bool CoralIntake::isBreakbeam2Broken(){
    return !(m_Breakbeam2.Get());
}

bool CoralIntake::isBreakbeam3Broken(){
    return !(m_Breakbeam3.Get());
}

bool CoralIntake::getintakeState(){
    return m_intakeState.Get();
}

void CoralIntake::RobotPeriodic(){
    if (isBreakbeamBroken() || isBreakbeam3Broken() || isBreakbeam3Broken()){
        intakeState = true;
    }

void CoralIntake::SetState(){
    intakeState = state;
}
};
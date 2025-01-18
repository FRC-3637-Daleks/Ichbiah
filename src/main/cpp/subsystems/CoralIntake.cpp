#include "subsystems/CoralIntake.h"

CoralIntake::CoralIntake(){
}

bool CoralIntake::getBreakbeamState(){
    return m_breakbeam.Get();
}
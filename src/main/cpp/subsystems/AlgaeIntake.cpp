#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake(){
}

bool AlgaeIntake::getBreakbeamState(){
    return m_breakbeam.Get();
}

void AlgaeIntake::moveForward(){
    m_AlgaeIntakeMotor.SetVoltage(12_V);
}

void AlgaeIntake::moveBackward(){
    m_AlgaeIntakeMotor.SetVoltage(-12_V);
}
   
void AlgaeIntake::stopMotor(){
    m_AlgaeIntakeMotor.SetVoltage(0_V);
}

frc2::CommandPtr AlgaeIntake::WhileIntake(){
    return frc2::cmd::RunEnd ([this]{ moveForward(); },
                              [this] {stopMotor(); });
}
frc2::CommandPtr AlgaeIntake::WhileOuttake(){
    return frc2::cmd::RunEnd ([this]{ moveBackward(); },
                              [this] {stopMotor(); });
}
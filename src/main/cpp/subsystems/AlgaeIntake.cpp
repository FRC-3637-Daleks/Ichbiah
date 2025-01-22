#include "subsystems/AlgaeIntake.h"

AlgaeIntake::AlgaeIntake(){
}

void AlgaeIntake::moveForward(){
    m_algaeIntakeMotor.SetVoltage(12_V);
}

void AlgaeIntake::moveBackward(){
    m_algaeIntakeMotor.SetVoltage(-12_V);
}
   
void AlgaeIntake::stopMotor(){
    m_algaeIntakeMotor.SetVoltage(0_V);
}

frc2::CommandPtr AlgaeIntake::WhileIntake(){
    return frc2::cmd::RunEnd ([this]{ moveForward(); },
                              [this] {stopMotor(); });
}
frc2::CommandPtr AlgaeIntake::WhileOuttake(){
    return frc2::cmd::RunEnd ([this]{ moveBackward(); },
                              [this] {stopMotor(); });
}
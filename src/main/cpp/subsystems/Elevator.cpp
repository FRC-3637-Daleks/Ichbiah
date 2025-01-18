#include "subsystems/Elevator.h"

Elevator::Elevator() {
    m_followerMotor.SetControl(ctre::phoenix6::controls::Follower{ElevatorConstants::leadmotorID, false});
};

void Elevator::MotorMoveUp() {
    m_leadMotor.SetVoltage(12_V);
};

void Elevator::MotorMoveDown() {
    m_leadMotor.SetVoltage(-12_V);
};

void Elevator::MotorStop() {
    m_leadMotor.SetVoltage(0_V);
};



#include "subsystems/Elevator.h"

Elevator::Elevator() {
    followerMotor.SetControl(ctre::phoenix6::controls::Follower{ElevatorConstants::leadmotorID, false});
};

void Elevator::MotorMoveUp() {
    leadMotor.SetVoltage(12_V);
};

void Elevator::MotorMoveDown() {
    leadMotor.SetVoltage(-12_V);
};

void Elevator::MotorStop() {
    leadMotor.SetVoltage(0_V);
};



#include "subsystems/Elevator.h"

Elevator::Elevator() {
    followerMotor.SetControl(ctre::phoenix6::controls::Follower{ElevatorConstants::leadmotorID, false});
    
    ElevatorConstants::kElevatorConfig.kP = 0.0;
    ElevatorConstants::kElevatorConfig.kD = 0.0;
    ElevatorConstants::kElevatorConfig.kI = 0.0;

    leadMotor.GetConfigurator().Apply(ElevatorConstants::kElevatorConfig);
    auto statusSignal = leadMotor.GetPosition();
    auto value = statusSignal.GetDataCopy().value() * 2 * std::numbers::pi * 3_cm;
};

void Elevator::SetMotorPosition() {
    ctre::phoenix6::controls::PositionVoltage m_request = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
    leadMotor.SetControl(m_request.WithPosition(10_tr));

}

void Elevator::MotorMoveUp() {
    leadMotor.SetVoltage(12_V);
};

void Elevator::MotorMoveDown() {
    leadMotor.SetVoltage(-12_V);
};

void Elevator::MotorStop() {
    leadMotor.SetVoltage(0_V);
};



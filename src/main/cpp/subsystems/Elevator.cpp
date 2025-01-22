#include "subsystems/Elevator.h"

Elevator::Elevator() {
    //Sets the follower motor
    m_followerMotor.SetControl(ctre::phoenix6::controls::Follower{ElevatorConstants::kLeadmotorID, false});
    
    //Sets and defines the Elevator motor PID config
    ElevatorConstants::kElevatorConfig.kP = ElevatorConstants::kP;
    ElevatorConstants::kElevatorConfig.kD = ElevatorConstants::kI;
    ElevatorConstants::kElevatorConfig.kI = ElevatorConstants::kD;
    m_leadMotor.GetConfigurator().Apply(ElevatorConstants::kElevatorConfig);

    //auto statusSignal = m_leadMotor.GetPosition();
    //auto value = statusSignal.GetDataCopy().value() * 2 * PI * 3_cm;
};

void Elevator::SetMotorPosition() {
    ctre::phoenix6::controls::PositionVoltage m_request = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
    m_leadMotor.SetControl(m_request.WithPosition(10_tr));
}

void Elevator::MotorMoveUp() {
    m_leadMotor.SetVoltage(12_V);
};

void Elevator::MotorMoveDown() {
    m_leadMotor.SetVoltage(-12_V);
};

void Elevator::MotorStop() {
    m_leadMotor.SetVoltage(0_V);
};



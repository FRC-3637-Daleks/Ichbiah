#include "subsystems/Elevator.h"
namespace ElevatorConstants {
    int kLeadmotorID = 50;
    int kFollowermotorID = 70;
    int kBreakBeamID = 60;

    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
}

Elevator::Elevator() : m_leadMotor{ElevatorConstants::kLeadmotorID},
                       m_followerMotor{ElevatorConstants::kFollowermotorID} {
    //Sets the follower motor
    using namespace ctre::phoenix6;

    //Sets and defines the Elevator motor PID config
    configs::TalonFXConfiguration m_ElevatorConfig;
    m_followerMotor.SetControl(controls::Follower{ElevatorConstants::kLeadmotorID, false});
    m_ElevatorConfig.WithSlot0(configs::Slot0Configs{}
                    .WithKP(ElevatorConstants::kP)
                    .WithKI(ElevatorConstants::kI)
                    .WithKD(ElevatorConstants::kD));
    m_leadMotor.GetConfigurator().Apply(m_ElevatorConfig);    
};

units::length::centimeter_t Elevator::GetEncoderPosition() {
    auto statusSignal = m_leadMotor.GetPosition();
    auto value = statusSignal.GetDataCopy().value() * 2 * std::numbers::pi * 3_cm;
    return value;
}

void Elevator::SetMotorPosition(units::angle::turn_t turns) {
    ctre::phoenix6::controls::PositionVoltage m_request = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
    m_leadMotor.SetControl(m_request.WithPosition(turns));
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

frc2::CommandPtr Elevator::WhileUp(){
    return frc2::cmd::RunEnd ([this]{MotorMoveUp(); },
                              [this] {MotorStop(); });
}

frc2::CommandPtr Elevator::WhileDown(){
    return frc2::cmd::RunEnd ([this]{MotorMoveDown(); },
                              [this] {MotorStop(); });
}
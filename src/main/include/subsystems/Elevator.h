#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#define PI 3.14159265359


namespace ElevatorConstants {
    ctre::phoenix6::configs::Slot0Configs kElevatorConfig{};

    int kLeadmotorID = 50;
    int kFollowermotorID = 70;
    int kBreakBeamID = 60;

    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
}

class Elevator {
    public:

    Elevator();

    void SetMotorPosition();

    void MotorMoveUp();
    void MotorMoveDown();
    void MotorStop();

    private:
    
    ctre::phoenix6::hardware::TalonFX m_leadMotor{ElevatorConstants::kLeadmotorID};
    ctre::phoenix6::hardware::TalonFX m_followerMotor{ElevatorConstants::kFollowermotorID};
};
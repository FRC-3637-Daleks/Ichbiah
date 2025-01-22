#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>


namespace ElevatorConstants {
    ctre::phoenix6::configs::Slot0Configs kElevatorConfig{};

    int leadmotorID = 50;
    int followermotorID = 70;
    int breakBeamID = 60;
}

class Elevator {
    public:

    Elevator();

    void MotorMoveUp();
    void MotorMoveDown();
    void MotorStop();
    void SetMotorPosition();

    private:

    ctre::phoenix6::hardware::TalonFX m_leadMotor{ElevatorConstants::leadmotorID};
    ctre::phoenix6::hardware::TalonFX m_followerMotor{ElevatorConstants::followermotorID};
};
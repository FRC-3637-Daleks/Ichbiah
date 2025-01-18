#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>


#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>


namespace ElevatorConstants {
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

    private:

    ctre::phoenix6::hardware::TalonFX m_leadMotor{ElevatorConstants::leadmotorID};
    ctre::phoenix6::hardware::TalonFX m_followerMotor{ElevatorConstants::followermotorID};
};
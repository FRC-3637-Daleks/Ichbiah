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

    ctre::phoenix6::CANBus kCANBus{"rio"};

    ctre::phoenix6::hardware::TalonFX leadMotor{ElevatorConstants::leadmotorID, kCANBus};
    ctre::phoenix6::hardware::TalonFX followerMotor{ElevatorConstants::followermotorID, kCANBus};
};
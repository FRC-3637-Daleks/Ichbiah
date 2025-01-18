#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>


#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>


namespace EndEffectorConstants {
    int motorID = 50;
    int breakBeamID = 60;
}

class EndEffector {
    public:

    EndEffector();

    void MotorForward();
    void MotorBack();
    bool getBreakBeamState();
    void MotorStop();

    private:
    frc::DigitalInput m_breakbeam{EndEffectorConstants::breakBeamID};

    ctre::phoenix6::CANBus kCANBus{"rio"};

    ctre::phoenix6::hardware::TalonFX endeffectorMotor{EndEffectorConstants::motorID, kCANBus};
};
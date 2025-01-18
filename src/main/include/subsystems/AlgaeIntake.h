#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>



namespace AlgaeIntakeConstants{
    int motorId = 10;
    int breakbeamID = 20;
}


class AlgaeIntake {
    public:

    AlgaeIntake();

    void moveForward();
    void moveBackward();
    void stopMotor();

    bool getBreakbeamState();

    private:
    
    frc::DigitalInput m_breakbeam{AlgaeIntakeConstants::breakbeamID};

    ctre::phoenix6::hardware::TalonFX m_AlgaeIntakeMotor{AlgaeIntakeConstants::motorId};
}; 
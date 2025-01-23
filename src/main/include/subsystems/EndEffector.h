#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>


namespace EndEffectorConstants {
    int kMotorID = 50;
    int kBreakBeamID = 60;
}

class EndEffector {
    public:

    EndEffector();

    void MotorForward();
    void MotorBack();
    void MotorStop();

    bool getBreakBeamState();

    frc2::CommandPtr WhileX();
    frc2::CommandPtr WhileNotX();

    private:
    frc::DigitalInput m_breakbeam{EndEffectorConstants::kBreakBeamID};

    ctre::phoenix6::hardware::TalonFX m_endEffectorMotor{EndEffectorConstants::kMotorID};
};
#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

class EndEffector {
    public:

    EndEffector();

    void MotorForward();
    void MotorBack();
    void MotorStop();

    bool isForwardBreakBeamBroken();
    bool isBackwardBreakBeamBroken();

    frc2::CommandPtr WhileOut();
    frc2::CommandPtr WhileIn();
    frc2::CommandPtr EffectorIn();
    frc2::CommandPtr EffectorContinue();
    frc2::CommandPtr EffectorOut();
    
    private:
    frc::DigitalInput m_ForwardBreakBeam;
    frc::DigitalInput m_BackwardBreakBeam;

    ctre::phoenix6::hardware::TalonFX m_endEffectorMotor;
};
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
    int kForwardBreakBeamID = 70;
    int kBackwardBreakBeamID = 80;
}

class EndEffector {
    public:

    EndEffector();

    void MotorForward();
    void MotorBack();
    void MotorStop();

    bool getBreakBeamState();
    bool isForwardBreakBeamBroken();
    bool isBackwardBreakBeamBroken();

    frc2::CommandPtr WhileOut();
    frc2::CommandPtr WhileIn();
    frc2::CommandPtr EffectorIn();
    frc2::CommandPtr EffectorContinue();
    frc2::CommandPtr EffectorOut();
    
    private:
    frc::DigitalInput m_breakbeam{EndEffectorConstants::kBreakBeamID};
    frc::DigitalInput m_ForwardBreakBeam{EndEffectorConstants::kForwardBreakBeamID};
    frc::DigitalInput m_BackwardBreakBeam{EndEffectorConstants::kBackwardBreakBeamID};

    ctre::phoenix6::hardware::TalonFX m_endEffectorMotor{EndEffectorConstants::kMotorID};
};
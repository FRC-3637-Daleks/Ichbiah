#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkFlex.h>

class EndEffector : public frc2::SubsystemBase{
    public:

    EndEffector();
    ~EndEffector();

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
    frc2::CommandPtr Intake();
    
    private:
    frc::DigitalInput m_ForwardBreakBeam;
    frc::DigitalInput m_BackwardBreakBeam;
    rev::spark::SparkFlex m_EndEffectorMotor;
};
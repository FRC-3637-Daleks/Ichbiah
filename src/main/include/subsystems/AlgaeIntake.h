#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

class AlgaeIntake : public frc2::SubsystemBase {
    public:

    AlgaeIntake();
    ~AlgaeIntake();
    
    bool isBreakbeamBroken();

    frc2::CommandPtr WhileIntake();
    frc2::CommandPtr WhileOuttake();
    frc2::CommandPtr IntakeIn();
    
    void moveForward(); 
    void moveBackward();
    void stopMotor();
    
    private: 
    frc::DigitalInput m_breakbeam;
    ctre::phoenix6::hardware::TalonFX m_algaeIntakeMotor;
}; 
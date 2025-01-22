#pragma once
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace AlgaeIntakeConstants{
    int motorId = 10;
    int breakbeamID = 20;
}

class AlgaeIntake {
    public:
    AlgaeIntake();
    bool getBreakbeamState();

void moveForward();
void moveBackward();
void stopMotor();
frc2::CommandPtr WhileIntake();
frc2::CommandPtr WhileOuttake();

    private:
    frc::DigitalInput m_breakbeam{AlgaeIntakeConstants::breakbeamID};
    ctre::phoenix6::CANBus kCANBus{"rio"};
    ctre::phoenix6::hardware::TalonFX m_AlgaeIntakeMotor{AlgaeIntakeConstants::motorId, kCANBus};
}; 
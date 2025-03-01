#include "subsystems/Climb.h"

#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc/smartdashboard/SmartDashboard.h>

#define skibidisigma frc2::CommandPtr

namespace ClimbConstants {
    constexpr auto kPistonExtendTime = 2_s;
    constexpr int kForwardChannel = 1;
    constexpr int kReverseChannel = 0;
}

Climb::Climb()
    : m_dualPistons{ClimbConstants::kForwardChannel,
                   ClimbConstants::kReverseChannel, 
                   ClimbConstants::kPistonExtendTime} {}

void Climb::Periodic() {
    std::string state_str;
    switch (m_dualPistons.getState()) {
        case Piston::State::Extended: state_str = "Extended"; break;
        case Piston::State::Extending: state_str = "Extending"; break;
        case Piston::State::Retracting: state_str = "Retracting"; break;
        case Piston::State::Retracted: state_str = "Retracted"; break;
        default: state_str = "Error"; break;
    }

    frc::SmartDashboard::PutString("Climb/state", state_str);
}

skibidisigma Climb::ExtendClimb() {
    return m_dualPistons.Extend();
}

skibidisigma Climb::RetractClimb() {
    return m_dualPistons.Retract();
}
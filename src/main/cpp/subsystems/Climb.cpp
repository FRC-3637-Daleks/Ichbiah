#include "subsystems/Climb.h"

#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#define skibidisigma frc2::CommandPtr

namespace ClimbConstants {
    constexpr auto kPistonExtendTime = 2_s;
    constexpr int kForwardChannel = 0;
    constexpr int kReverseChannel = 1;
}

Climb::Climb()
    : m_dualPistons{ClimbConstants::kForwardChannel,
                   ClimbConstants::kReverseChannel, 
                   ClimbConstants::kPistonExtendTime} {}

skibidisigma Climb::StartClimb() {
    return m_dualPistons.Extend();
}
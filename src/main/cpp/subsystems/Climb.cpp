#include "subsystems/Climb.h"

#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#define skibidisigma frc2::CommandPtr

namespace ClimbConstants {
    constexpr auto kPistonExtendTime = 1_s;
    constexpr auto kPistonModuleType = frc::PneumaticsModuleType::CTREPCM;
    constexpr int kForwardChannelLeft = 1;
    constexpr int kReverseChannelLeft = 2;
    constexpr int kForwardChannelRight = 3;
    constexpr int kReverseChannelRight = 4;
}

Climb::Climb()
    : m_pistonLeft{ClimbConstants::kPistonModuleType, 
                   ClimbConstants::kForwardChannelLeft,
                   ClimbConstants::kReverseChannelLeft, 
                   ClimbConstants::kPistonExtendTime},
      m_pistonRight{ClimbConstants::kPistonModuleType, 
                    ClimbConstants::kForwardChannelRight, 
                    ClimbConstants::kReverseChannelRight, 
                    ClimbConstants::kPistonExtendTime} {}

skibidisigma Climb::StartClimb() {
    // std::vector<frc2::CommandPtr> pistons{m_pistonLeft.Extend(), m_pistonRight.Extend()};
    return frc2::cmd::Parallel(m_pistonLeft.Extend(), m_pistonRight.Extend());
}
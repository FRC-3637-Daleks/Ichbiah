#include "subsystems/Climb.h"

#include <vector>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ClimbConstants {
constexpr auto kPistonExtendTime = 2_s;
constexpr int kForwardChannel = 0;
constexpr int kReverseChannel = 1;

constexpr int kModualID = 3; // Remember to change to real value

constexpr units::pounds_per_square_inch_t kMaxPressure = 120_psi;

constexpr units::pounds_per_square_inch_t kMinPressure = 80_psi;
} // namespace ClimbConstants

Climb::Climb()
    : m_dualPistons{ClimbConstants::kForwardChannel,
                    ClimbConstants::kReverseChannel,
                    ClimbConstants::kPistonExtendTime},
      m_compressor{
          ClimbConstants::kModualID,
          frc::PneumaticsModuleType::CTREPCM /*check, proob not right*/} {
  //"Enable closed-loop mode based on both the digital pressure switch AND the
  // analog pressure sensor connected to the PH." - WPILib Docs
  m_compressor.EnableHybrid(ClimbConstants::kMinPressure,
                            ClimbConstants::kMaxPressure);
}

void Climb::Periodic() {
  std::string state_str;
  switch (m_dualPistons.getState()) {
  case Piston::State::Extended:
    state_str = "Extended";
    break;
  case Piston::State::Extending:
    state_str = "Extending";
    break;
  case Piston::State::Retracting:
    state_str = "Retracting";
    break;
  case Piston::State::Retracted:
    state_str = "Retracted";
    break;
  default:
    state_str = "Error";
    break;
  }

  frc::SmartDashboard::PutString("Climb/state", state_str);
}

frc2::CommandPtr Climb::ExtendClimb() { return m_dualPistons.Extend(); }

frc2::CommandPtr Climb::RetractClimb() { return m_dualPistons.Retract(); }
#include "subsystems/CoralIntake.h"

namespace CoralIntakeConstants {
constexpr int kBreakbeamID = 3;
constexpr int kBreakbeam2ID = 4;
constexpr int kBreakbeam3ID = 5;
constexpr int kintakeStateID = 6;
} // namespace CoralIntakeConstants
CoralIntake::CoralIntake()
    : m_Breakbeam{CoralIntakeConstants::kBreakbeamID},
      m_Breakbeam2{CoralIntakeConstants::kBreakbeam2ID},
      m_Breakbeam3{CoralIntakeConstants::kBreakbeam3ID},
      m_intakeState{CoralIntakeConstants::kintakeStateID}, intakeState{false} {}

bool CoralIntake::getBreakbeamState() { return m_Breakbeam.Get(); }

bool CoralIntake::isBreakbeamBroken() { return !(m_Breakbeam.Get()); }

bool CoralIntake::isBreakbeam2Broken() { return !(m_Breakbeam2.Get()); }

bool CoralIntake::isBreakbeam3Broken() { return !(m_Breakbeam3.Get()); }

bool CoralIntake::getintakeState() { return m_intakeState.Get(); }
void CoralIntake::SetState(bool state) { intakeState = state; }
bool CoralIntake::GetState() { return intakeState; }
void CoralIntake::Periodic() {
  if (isBreakbeamBroken() || isBreakbeam3Broken() || isBreakbeam3Broken()) {
    intakeState = true;
  }
};

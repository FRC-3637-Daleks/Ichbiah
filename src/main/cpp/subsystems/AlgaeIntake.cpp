#include "subsystems/AlgaeIntake.h"

namespace AlgaeIntakeConstants {
int kMotorId = 10;
int kBreakbeamID = 20;
} // namespace AlgaeIntakeConstants

AlgaeIntake::AlgaeIntake()
    : m_breakbeam{AlgaeIntakeConstants::kBreakbeamID},
      m_algaeIntakeMotor{AlgaeIntakeConstants::kMotorId} {};

void AlgaeIntake::moveForward() { m_algaeIntakeMotor.SetVoltage(12_V); }

void AlgaeIntake::moveBackward() { m_algaeIntakeMotor.SetVoltage(-12_V); }

void AlgaeIntake::stopMotor() { m_algaeIntakeMotor.SetVoltage(0_V); }

frc2::CommandPtr AlgaeIntake::WhileIntake() {
  return RunEnd([this] { moveForward(); }, [this] { stopMotor(); });
}
frc2::CommandPtr AlgaeIntake::WhileOuttake() {
  return RunEnd([this] { moveBackward(); }, [this] { stopMotor(); });
}

frc2::CommandPtr AlgaeIntake::IntakeIn() {
  return Run([this] { IntakeIn(); }).Until([this]() -> bool {
    return isBreakbeamBroken();
  });
}

AlgaeIntake::~AlgaeIntake() {}

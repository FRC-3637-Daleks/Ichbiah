#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/time.h>

class Piston : public frc2::SubsystemBase {

public:
  enum State { Extending, Extended, Retracting, Retracted };

  Piston(int fChannel, int rChannel, units::second_t delay);
  frc2::CommandPtr Extend();
  frc2::CommandPtr Retract();
  frc2::CommandPtr Off();
  State getState();

private:
  frc::DoubleSolenoid m_solenoid;
  units::second_t stroke_delay;

  State m_state;
};

#include "subsystems/Piston.h"

namespace PistonConstants {
constexpr int kModuleID = 3;
}; // namespace PistonConstants

Piston::Piston(int forwardChannel, int reverseChannel, units::second_t delay)
    : m_solenoid{PistonConstants::kModuleID, frc::PneumaticsModuleType::CTREPCM,
                 forwardChannel, reverseChannel},
      stroke_delay{delay}, m_state{State::Retracted} {}

frc2::CommandPtr Piston::Extend() {
  return Run([this] {
           m_state = State::Extending;
           m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
         })
      .WithTimeout(stroke_delay)
      .AndThen([this] { m_state = State::Extended; });
}

frc2::CommandPtr Piston::Retract() {
  return Run([this] {
           m_state = State::Retracting;
           m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);
         })
      .WithTimeout(stroke_delay)
      .AndThen([this] { m_state = State::Retracted; });
}

frc2::CommandPtr Piston::Off() {
  return Run([this] { m_solenoid.Set(frc::DoubleSolenoid::Value::kOff); });
}

Piston::State Piston::getState() { return m_state; }
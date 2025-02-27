#include "subsystems/Piston.h"

Piston::Piston(int forwardChannel, int reverseChannel, units::second_t delay)
: m_solenoid{frc::PneumaticsModuleType::REVPH,
            forwardChannel, 
            reverseChannel},
  stroke_delay{delay} {

    m_state = State::Retracted;
}

frc2::CommandPtr Piston::Extend() {
    
    return frc2::cmd::Run([this]{m_state = State::Extending;
    m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);})
    .AndThen(frc2::cmd::Wait(stroke_delay))
    .AndThen([this]{m_state = State::Extended;});
}

frc2::CommandPtr Piston::Retract() {
    return frc2::cmd::Run([this]{m_state = State::Retracting;
    m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);})
    .AndThen(frc2::cmd::Wait(stroke_delay))
    .AndThen([this]{m_state = State::Retracted;});
}

frc2::CommandPtr Piston::Off() {
    return frc2::cmd::Run([this]{m_solenoid.Set(frc::DoubleSolenoid::Value::kOff);});
}

inline Piston::State Piston::getState() {
    return m_state;
}
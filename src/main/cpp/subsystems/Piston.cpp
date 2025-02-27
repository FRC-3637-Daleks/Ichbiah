#include "subsystems/Piston.h"

Piston::Piston(int forwardChannel, int reverseChannel, units::second_t delay)
: m_solenoid{frc::PneumaticsModuleType::REVPH,
            forwardChannel, 
            reverseChannel},
  stroke_delay{delay} {

    m_state = State::Retracted;
}

frc2::CommandPtr Piston::Extend() {
    
    return Run(
        [this]{
            m_state = State::Extending;
            m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);
        })
        .WithTimeout(stroke_delay)
        .AndThen([this]{m_state = State::Extended;});
}

frc2::CommandPtr Piston::Retract() {
    return Run(
        [this]{
            m_state = State::Retracting;
            m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);
        })
        .WithTimeout(stroke_delay)
        .AndThen([this]{m_state = State::Retracted;});
}

frc2::CommandPtr Piston::Off() {
    return Run([this]{m_solenoid.Set(frc::DoubleSolenoid::Value::kOff);});
}

Piston::State Piston::getState() {
    return m_state;
}
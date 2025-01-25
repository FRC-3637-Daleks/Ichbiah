#include "subsystems/Piston.h"

//Forward Channal = ID 1, Reverse Channal = ID 2
Piston::Piston(frc::PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel, units::seconds_t delay)
: m_solenoid{moduleType, forwardChannel, reverseChannel}, stroke_delay{delay} {}

frc2::CommandPtr Piston::Extend() {
    m_state = Extending;
    return frc::cmd::Run([this]{m_solenoid.Set(frc::DoubleSolenoid::Value::kForward);}.AndThen(WaitCommand(m_solenoid)).AndThen([this]{m_state = Extended}));
}

frc2::CommandPtr Piston::Retract() {
    m_state = Retracting;
    return frc::cmd::Run([this]{m_solenoid.Set(frc::DoubleSolenoid::Value::kReverse);}.AndThen(WaitCommand(m_solenoid)).AndThen(m_state = Retracted));
}

frc2::CommandPtr Piston::Off() {
    m_solenoid.Set(frc::DoubleSolenoid::Value::kOff);
    return frc::cmd::Run([this]{m_solenoid.Set(frc::DoubleSolenoid::Value::kOff);});

}

bool Piston::isExtended() {
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool Piston::isRetracted() {
    return m_solenoid.Get() == frc::DoubleSolenoid::Value::kReverse;
}
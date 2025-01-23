#include "subsystems/EndEffector.h"

EndEffector::EndEffector() {

};

void EndEffector::MotorForward() {
    m_endEffectorMotor.SetVoltage(12_V);
};

void EndEffector::MotorBack() {
    m_endEffectorMotor.SetVoltage(-12_V);
};

void EndEffector::MotorStop() {
    m_endEffectorMotor.SetVoltage(0_V);
};

bool EndEffector::getBreakBeamState() {
    return m_breakbeam.Get();
};

frc2::CommandPtr EndEffector::WhileX(){
    return frc2::cmd::RunEnd ([this]{ EndEffector::MotorForward(); },
                              [this] {EndEffector::MotorStop(); });
}
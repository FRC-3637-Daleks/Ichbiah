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

frc2::CommandPtr EndEffector::WhileIn(){
    return frc2::cmd::RunEnd ([this]{ EndEffector::MotorForward(); },
                              [this] {EndEffector::MotorStop(); });
}

bool EndEffector::isForwardBreakBeamBroken(){
    return !(m_ForwardBreakBeam.Get());
}
bool EndEffector::isBackwardBreakBeamBroken(){
    return !(m_BackwardBreakBeam.Get());
}

frc2::CommandPtr EndEffector::EffectorIn() {
    return frc2::cmd::Run([this] { WhileIn(); })
    .Until([this]() -> bool {
        return isForwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorContinue() {
    return frc2::cmd::Run([this] { WhileIn(); })
    .Until([this]() -> bool {
        return !isBackwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorOut() {
    return frc2::cmd::Run([this] { WhileOut(); })
    .Until([this]() -> bool {
        return !isForwardBreakBeamBroken();
    });
}
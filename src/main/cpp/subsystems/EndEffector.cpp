#pragma once
#include "subsystems/EndEffector.h"

namespace EndEffectorConstants {
    int kMotorID = 50;
    int kBreakBeamID = 60;
    int kForwardBreakBeamID = 70;
    int kBackwardBreakBeamID = 80;
}

EndEffector::EndEffector():
                        m_ForwardBreakBeam{EndEffectorConstants::kForwardBreakBeamID},
                        m_BackwardBreakBeam{EndEffectorConstants::kBackwardBreakBeamID},                  
                        m_endEffectorMotor{EndEffectorConstants::kMotorID}
{

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

frc2::CommandPtr EndEffector::WhileIn(){
    return frc2::cmd::RunEnd ([this]{ EndEffector::MotorForward(); },
                              [this] {EndEffector::MotorStop(); });
}

frc2::CommandPtr EndEffector::WhileOut(){
    return frc2::cmd::RunEnd ([this]{ EndEffector::MotorBack(); },
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
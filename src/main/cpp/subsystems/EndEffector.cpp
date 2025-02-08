#include "subsystems/EndEffector.h"

namespace EndEffectorConstants {
    int kMotorID = 50;
    int kBreakBeamID = 60;
    int kForwardBreakBeamID = 70;
    int kBackwardBreakBeamID = 80;
}

EndEffector::EndEffector() :
    m_BackwardBreakBeam(EndEffectorConstants::kBackwardBreakBeamID),
    m_ForwardBreakBeam{EndEffectorConstants::kForwardBreakBeamID},
    m_EndEffectorMotor{EndEffectorConstants::kMotorID}
{
};

void EndEffector::MotorForward() {
    m_EndEffectorMotor.SetVoltage(12_V);
};

void EndEffector::MotorBack() {
    m_EndEffectorMotor.SetVoltage(-12_V);
};

void EndEffector::MotorStop() {
    m_EndEffectorMotor.SetVoltage(0_V);
};


frc2::CommandPtr EndEffector::WhileIn(){
    return RunEnd([this]{ EndEffector::MotorForward(); },
                              [this] {EndEffector::MotorStop(); });
}

bool EndEffector::isForwardBreakBeamBroken(){
    return !(m_ForwardBreakBeam.Get());
}
bool EndEffector::isBackwardBreakBeamBroken(){
    return !(m_BackwardBreakBeam.Get());
}

frc2::CommandPtr EndEffector::EffectorIn() {
    return Run([this] { WhileIn(); })
    .Until([this]() -> bool {
        return isForwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorContinue() {
    return Run([this] { WhileIn(); })
    .Until([this]() -> bool {
        return !isBackwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorOut() {
    return Run([this] { WhileOut(); })
    .Until([this]() -> bool {
        return !isForwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::WhileIn(){
    return Run([this]{WhileIn();})
    .Until([this]() -> bool {
        return isForwardBreakBeamBroken();
    });
} 
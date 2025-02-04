#include "subsystems/EndEffector.h"

/**
 * Note from Visvam:
 * 
 * Use constexpr for IDs. It helps the robot code run faster and use less memory.
 * 
 * Also remember that there are only 10 DIO ports on the RoboRIO 2.0.
 */

namespace EndEffectorConstants {
    constexpr int kMotorID = 5;
    constexpr int kBreakBeamID = 6;
    constexpr int kForwardBreakBeamID = 7;
    constexpr int kBackwardBreakBeamID = 8;
}

EndEffector::EndEffector() :
    m_ForwardBreakBeam{EndEffectorConstants::kForwardBreakBeamID},
    m_BackwardBreakBeam(EndEffectorConstants::kBackwardBreakBeamID),
    m_EndEffectorMotor{EndEffectorConstants::kMotorID,rev::spark::SparkFlex::MotorType::kBrushless}
{}

/**
 * Note from Visvam:
 * 
 * Don't for get to define functions you declare in your header file!
 * 
 * You also don't need to add semicolons after function definitions.
 */
EndEffector::~EndEffector() {}

void EndEffector::MotorForward() {
    m_EndEffectorMotor.SetVoltage(12_V);
}

void EndEffector::MotorBack() {
    m_EndEffectorMotor.SetVoltage(-12_V);
}

void EndEffector::MotorStop() {
    m_EndEffectorMotor.SetVoltage(0_V);
}


frc2::CommandPtr EndEffector::WhileIn(){
    return RunEnd([this]{ EndEffector::MotorForward(); },
                  [this] {EndEffector::MotorStop(); });
}

/**
 * Note from Visvam:
 * 
 * This function was being called without a definition. I wrote this code based on how you wrote WhileIn().
 * 
 * Please change it to work how you see fit.
 */

frc2::CommandPtr EndEffector::WhileOut() {
    return RunEnd(
        [this] () { MotorBack(); },
        [this] () { MotorStop(); }
    );
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

frc2::CommandPtr EndEffector::Intake(){
    return Run([this]{WhileIn();})
    .Until([this]() -> bool {
        return isForwardBreakBeamBroken();
    });
}

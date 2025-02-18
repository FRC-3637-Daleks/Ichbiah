#include "subsystems/EndEffector.h"

#include <rev/sim/SparkFlexSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/DIOSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/moment_of_inertia.h>
#include <frc/RobotController.h>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

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

    // Sensor is low when the beam is obstructed
    constexpr bool kBeamBroken = false;

    constexpr auto kRollerMass = 0.25_lb;  // just need ballpark numbers
    constexpr auto kCoralMass = 1_lb;  // sure toss this into the mix "realism"
    constexpr auto kRollerDiameter = 1.5_in;
    constexpr auto kEEMoment = 0.5*(kRollerMass+kCoralMass)*units::math::pow<2>(kRollerDiameter/2);
    constexpr auto kEEGearing = 1.0;
    constexpr auto kRollerCircum = kRollerDiameter * std::numbers::pi;
    constexpr auto kCoralLength = 12_in;
    constexpr auto kBackBeamPos = -2_in;
    constexpr auto kFrontBeamPos = 2_in;
}

class EndEffectorSim {
public:
    friend class EndEffector;

public:
    EndEffectorSim(EndEffector& ee);

public:
    frc::DCMotor m_motor;
    rev::spark::SparkFlexSim m_motor_sim;
    frc::sim::FlywheelSim m_ee_model;
    frc::sim::DIOSim m_fwd_bb_sim, m_back_bb_sim;

    units::inch_t m_coral_pos;
    bool m_has_coral;
};

EndEffector::EndEffector() :
    m_ForwardBreakBeam{EndEffectorConstants::kForwardBreakBeamID},
    m_BackwardBreakBeam(EndEffectorConstants::kBackwardBreakBeamID),
    m_EndEffectorMotor{EndEffectorConstants::kMotorID,rev::spark::SparkFlex::MotorType::kBrushless},
    m_sim_state(new EndEffectorSim(*this))
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

/*****************************SIMULATION******************************/
EndEffectorSim::EndEffectorSim(EndEffector& ee):
    m_motor(frc::DCMotor::NeoVortex(1)),
    m_motor_sim(&ee.m_EndEffectorMotor, &m_motor),
    m_ee_model(
        frc::LinearSystemId::FlywheelSystem(
            m_motor,
            EndEffectorConstants::kEEMoment,
            EndEffectorConstants::kEEGearing),
        m_motor),
    m_fwd_bb_sim(ee.m_ForwardBreakBeam),
    m_back_bb_sim(ee.m_BackwardBreakBeam),
    m_coral_pos(0),
    m_has_coral(false) {
}

void EndEffector::SimulationPeriodic() {
    if (!m_sim_state) return;

    constexpr auto dt = 20_ms;
    using namespace EndEffectorConstants;

    units::volt_t bus_voltage{frc::RobotController::GetInputVoltage()};
    m_sim_state->m_ee_model.SetInputVoltage(
        m_sim_state->m_motor_sim.GetAppliedOutput() * bus_voltage);
    
    m_sim_state->m_ee_model.Update(dt);

    const auto ee_vel = m_sim_state->m_ee_model.GetAngularVelocity();

    m_sim_state->m_motor_sim.iterate(
        units::revolutions_per_minute_t{ee_vel}.value(), 
        bus_voltage.value(),
        units::second_t{dt}.value()
    );

    if (m_sim_state->m_has_coral) {
        m_sim_state->m_coral_pos += ee_vel * kRollerCircum / 1_tr * dt;

        if (m_sim_state->m_coral_pos < 0_in)
            m_sim_state->m_coral_pos = 0_in;
        if (m_sim_state->m_coral_pos > kCoralLength) {
            m_sim_state->m_has_coral = false;
            m_sim_state->m_coral_pos = 0_in;
        }

        m_sim_state->m_back_bb_sim.SetValue(
            (m_sim_state->m_coral_pos - kCoralLength < kBackBeamPos) == kBeamBroken);
        m_sim_state->m_fwd_bb_sim.SetValue(
            (m_sim_state->m_coral_pos > kFrontBeamPos) == kBeamBroken);
    } else {
        m_sim_state->m_back_bb_sim.SetValue(!kBeamBroken);
        m_sim_state->m_fwd_bb_sim.SetValue(!kBeamBroken);
    }

    frc::SmartDashboard::PutNumber("EndEffector/sim coral inches", 
        m_sim_state->m_has_coral? m_sim_state->m_coral_pos.value():-1.0);
}

void EndEffector::SimulateNewCoral() {
    if (!m_sim_state || m_sim_state->m_has_coral) return;
    m_sim_state->m_has_coral = true;
    m_sim_state->m_coral_pos = -1_in;
}

void EndEffector::SimulatePreloadCoral() {
    if (!m_sim_state) return;
    m_sim_state->m_has_coral = true;
    m_sim_state->m_coral_pos = 4_in; // sure why not
}

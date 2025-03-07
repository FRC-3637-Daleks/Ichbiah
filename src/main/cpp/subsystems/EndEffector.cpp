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
    constexpr int kMotorID = 2;
    constexpr int kForwardBreakBeamID = 1;
    constexpr int kBackwardBreakBeamID = 2;

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
    m_sim_state(new EndEffectorSim(*this)) {
    
    rev::spark::SparkBaseConfig config;
    config
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(30)
        .OpenLoopRampRate(0.2);
    config.limitSwitch
        .ForwardLimitSwitchEnabled(false)
        .ReverseLimitSwitchEnabled(false);
            
    m_EndEffectorMotor.Configure(
        config,
        rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kPersistParameters
    );

}

/**
 * Note from Visvam:
 * 
 * Don't for get to define functions you declare in your header file!
 * 
 * You also don't need to add semicolons after function definitions.
 */
EndEffector::~EndEffector() {}

void EndEffector::Periodic() {
    UpdateDashboard();
}

void EndEffector::UpdateDashboard() {
    frc::SmartDashboard::PutBoolean("EndEffector/has coral?", hasCoral());
    frc::SmartDashboard::PutNumber("EndEffector/output", m_EndEffectorMotor.GetAppliedOutput());
    UpdateVisualization();
}

void EndEffector::InitVisualization(frc::MechanismObject2d *elevator_end) {
    if (!elevator_end) return;

    m_mech_endeffector_base = elevator_end
        ->Append<frc::MechanismLigament2d>("endeffector_base", 0.4, 0_deg, 4, frc::Color::kGreen)
        ->Append<frc::MechanismLigament2d>("endeffector_ramp", 0.0, -90_deg - 35_deg, 4, frc::Color::kGreen);
    
    m_mech_endeffector_base->Append<frc::MechanismLigament2d>(
        "endeffector_intake", 0.5, 180_deg, 4, frc::Color::kGreen);
    m_mech_endeffector_base->Append<frc::MechanismLigament2d>(
        "endeffector_outtake", 0.5, 0_deg, 4, frc::Color::kGreen);
    
    m_mech_backbeam = m_mech_endeffector_base->Append<frc::MechanismLigament2d>(
        "backbeam", 0.0, 180_deg, 16, frc::Color::kWhiteSmoke);
    m_mech_frontbeam = m_mech_endeffector_base->Append<frc::MechanismLigament2d>(
        "frontbeam", 0.0, 0_deg, 16, frc::Color::kWhiteSmoke);
    
    m_mech_spinner = m_mech_endeffector_base->Append<frc::MechanismLigament2d>(
        "spinner_strut", 0.3, 90_deg, 2, frc::Color::kGreen)->Append<frc::MechanismLigament2d>(
            "spinner", 0.2, 10_deg, 3, frc::Color::kGreenYellow);
}

void EndEffector::UpdateVisualization() {
    if (!m_mech_backbeam) return;

    bool back = isBackwardBreakBeamBroken();
    bool front = isForwardBreakBeamBroken();
    double back_length = 0.0, front_length = 0.0;

    if (back && front) back_length = front_length = 0.5;
    else if (back) back_length = 1.0;
    else if (front) front_length = 1.0;

    m_mech_backbeam->SetLength(back_length);
    m_mech_frontbeam->SetLength(front_length);
    
    m_mech_spinner->SetAngle(1_deg*(m_mech_spinner->GetAngle() + m_EndEffectorMotor.GetAppliedOutput() * 200));
}

void EndEffector::MotorForward() {
    m_EndEffectorMotor.SetVoltage(3_V);
}

void EndEffector::MotorForward2() {
    m_EndEffectorMotor.SetVoltage(1_V);
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

frc2::CommandPtr EndEffector::WhileIn2(){
    return RunEnd([this]{ EndEffector::MotorForward2(); },
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

bool EndEffector::hasCoral() {
    return isForwardBreakBeamBroken() || isBackwardBreakBeamBroken();
}

/*
             ______  _ Motor
__                  \x|_______
   \ ______  X Front Break Beam
            \ ______
                     \ ____ X Back Break Beam
                            \ ____
                                    ||
                                    ||
                                    || Reef
 */

frc2::CommandPtr EndEffector::EffectorIn() {
    return WhileIn().Until([this]() -> bool {
         return isForwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorContinue() {
    return WhileIn().Until([this]() -> bool {
        return !isBackwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::EffectorOut() {
    return WhileOut().Until([this]() -> bool {
        return !isForwardBreakBeamBroken();
    });
}

frc2::CommandPtr EndEffector::Intake(){
    return WhileIn().Until([this]() -> bool {
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

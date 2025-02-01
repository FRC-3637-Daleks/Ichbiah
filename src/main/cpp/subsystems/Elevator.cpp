#include "subsystems/Elevator.h"

#include <frc/simulation/ElevatorSim.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace ElevatorConstants {
// Device Addresses
    int kLeadmotorID = 50;
    int kFollowermotorID = 70;
    int kBottomLimitSwitchID = 60;

// Physical Parameters
    constexpr auto kSpoolRadius = 1.7_in;  // Estimated, confirm with mechanical
    constexpr auto kSpoolCircum = kSpoolRadius * 2 * std::numbers::pi;
    constexpr auto kGearReduction = 5.0;  // Tentative, could change
    constexpr auto kMinHeight = 2_ft;  // VERY estimated, confirm with CAD
    constexpr auto kMaxHeight = 6_ft;  // VERY estimated, confirm with CAD
    constexpr auto kMass = 9_kg;    // Guess-value, not particularly important, edit once built

//Level Height
    
    constexpr units::length::centimeter_t goal_heights[] = {0_cm, 90_cm, 127_cm, 150_cm, 180_cm};

    constexpr units::length::centimeter_t kL1 = 90_cm;
    constexpr units::length::centimeter_t kL2 = 127_cm;
    constexpr units::length::centimeter_t kL3 = 150_cm;
    constexpr units::length::centimeter_t kL4 = 180_cm;
    constexpr units::length::centimeter_t kTolerance = 2_cm;

// Feedback/Feedforward Gains
    double kP = 1.0;
    double kI = 0.0007;
    double kD = 0.06;
    double kG = 0.04053;
}

class ElevatorSim {
public:
    friend class Elevator;

public:
    ElevatorSim(Elevator& elevator);

public:
    // models the elevator
    frc::sim::ElevatorSim m_elevatorModel;

    // accesses internals of the talon FX objects to inject the simulated data
    ctre::phoenix6::sim::TalonFXSimState m_leadSim, m_followerSim;
};

Elevator::Elevator() : m_leadMotor{ElevatorConstants::kLeadmotorID},
                       m_followerMotor{ElevatorConstants::kFollowermotorID},
                       m_sim_state{new ElevatorSim{*this}} {
    //Sets the follower motor
    using namespace ctre::phoenix6;

    //Sets and defines the Elevator motor PID config
    configs::TalonFXConfiguration m_ElevatorConfig;

    ctre::phoenix6::configs::HardwareLimitSwitchConfigs LimitConfig{};
    LimitConfig.ReverseLimitAutosetPositionEnable = true;
    LimitConfig.ReverseLimitAutosetPositionValue = 0_tr;
    LimitConfig.ReverseLimitRemoteSensorID = ElevatorConstants::kBottomLimitSwitchID;


    m_followerMotor.SetControl(controls::Follower{ElevatorConstants::kLeadmotorID, false});
    m_ElevatorConfig.WithSlot0(configs::Slot0Configs{}
                    .WithKP(ElevatorConstants::kP)
                    .WithKI(ElevatorConstants::kI)
                    .WithKD(ElevatorConstants::kD)
                    .WithKG(ElevatorConstants::kG))
                    .WithHardwareLimitSwitch(LimitConfig);
    m_leadMotor.GetConfigurator().Apply(m_ElevatorConfig);    
};

void Elevator::GoToLevel(Elevator::Level level) {
    goalLevel = level;
};

bool Elevator::IsAtPos(units::length::centimeter_t pos) {
    return (units::math::abs((pos - GetEncoderPosition())) <= (ElevatorConstants::kTolerance));
};

bool Elevator::IsAtLevel(Elevator::Level level) {
    return IsAtPos(ElevatorConstants::goal_heights[level] + ElevatorConstants::kMinHeight);
}


units::length::centimeter_t Elevator::GetEncoderPosition() {
    auto statusSignal = m_leadMotor.GetPosition();
    auto value = statusSignal.GetDataCopy().value() * 2 * std::numbers::pi * 3_cm;
    return value;
}

void Elevator::SetMotorPosition(units::length::centimeter_t length) {
    length -= ElevatorConstants::kMinHeight;
    ctre::phoenix6::controls::PositionVoltage m_request = 
        ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0)
        .WithLimitReverseMotion(m_reverseLimit.Get());
    m_leadMotor.SetControl(m_request.WithPosition((units::angle::turn_t)(length / ElevatorConstants::kSpoolCircum * 1_tr).value()));
}

void Elevator::SetMotorPosition(Elevator::Level level) {
    SetMotorPosition(ElevatorConstants::goal_heights[level] + ElevatorConstants::kMinHeight);
}

void Elevator::MotorMoveUp() {
    m_leadMotor.SetVoltage(12_V);
};

void Elevator::MotorMoveDown() {
    m_leadMotor.SetVoltage(-12_V);
};

void Elevator::MotorStop() {
    m_leadMotor.SetVoltage(0_V);
};

frc2::CommandPtr Elevator::WhileUp(){
    return frc2::cmd::RunEnd ([this]{MotorMoveUp(); },
                              [this] {MotorStop(); });
}

frc2::CommandPtr Elevator::WhileDown(){
    return frc2::cmd::RunEnd ([this]{MotorMoveDown(); },
                              [this] {MotorStop(); });
}

void Elevator::RobotPeriodic() {
    SetMotorPosition(goalLevel);
}
//***************************SIMULATION*****************************
ElevatorSim::ElevatorSim(Elevator& elevator):
    m_elevatorModel{
        frc::DCMotor::KrakenX60FOC(2),
        ElevatorConstants::kGearReduction,
        ElevatorConstants::kMass,
        ElevatorConstants::kSpoolRadius,
        ElevatorConstants::kMinHeight,
        ElevatorConstants::kMaxHeight,
        true,   // simulate gravity
        ElevatorConstants::kMinHeight  // starting height
    },
    m_leadSim{elevator.m_leadMotor},
    m_followerSim{elevator.m_followerMotor}
{
}

void Elevator::SimulationPeriodic() {
    if (!m_sim_state) return;

    // reduce code clutter
    auto &m_elevatorModel = m_sim_state->m_elevatorModel;
    auto &m_leadSim = m_sim_state->m_leadSim;
    auto &m_followerSim = m_sim_state->m_followerSim;

    const auto supply_voltage = frc::RobotController::GetBatteryVoltage();
    m_leadSim.SetSupplyVoltage(supply_voltage);
    m_followerSim.SetSupplyVoltage(supply_voltage);

    // Set inputs into model
    m_elevatorModel.SetInputVoltage(m_leadSim.GetMotorVoltage());
    
    // Simulate the model over the next 20ms
    m_elevatorModel.Update(20_ms);

    // The motor turns kGearReduction times to turn the spool once
    // This raises the elevator one spool circumference up
    constexpr auto rotor_turns_per_elevator_height = 
        ElevatorConstants::kGearReduction * 1_tr / ElevatorConstants::kSpoolCircum;
    
    // Feed simulated outputs of model back into user program
    const auto position = m_elevatorModel.GetPosition();
    const units::turn_t rotor_turns =
        (position - ElevatorConstants::kMinHeight) * rotor_turns_per_elevator_height;

    const auto velocity = m_elevatorModel.GetVelocity();
    const units::turns_per_second_t rotor_velocity =
        velocity * rotor_turns_per_elevator_height;
    
    m_leadSim.SetRawRotorPosition(rotor_turns);
    m_leadSim.SetRotorVelocity(rotor_velocity);
    m_leadSim.SetForwardLimit(m_elevatorModel.HasHitUpperLimit());
    m_leadSim.SetReverseLimit(m_elevatorModel.HasHitLowerLimit());

    // mechanically linked, though we should never read this value
    m_followerSim.SetRawRotorPosition(rotor_turns);
    m_followerSim.SetRotorVelocity(rotor_velocity);

    //Publishing data to NetworkTables
    frc::SmartDashboard::PutNumber("Elevator/Position ", m_leadSim.GetMotorVoltage().value());
}

Elevator::~Elevator() {}
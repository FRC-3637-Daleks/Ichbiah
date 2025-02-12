#include "subsystems/Elevator.h"

#include <frc/simulation/ElevatorSim.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/DIOSim.h>

#include <random>

namespace ElevatorConstants {
// Device Addresses
    int kLeadmotorID = 50;
    int kFollowermotorID = 70;
    int kBottomLimitSwitchID = 60;
    int kReverseLimitID = 1;
    int kForwardLimitID = 0;

// Physical Parameters
    constexpr auto kSpoolRadius = 1.7_in;  // Estimated, confirm with mechanical
    constexpr auto kSpoolCircum = kSpoolRadius * 2 * std::numbers::pi;
    constexpr auto kGearReduction = 8.5;  // Tentative, could change
    constexpr auto kMinHeight = 1_ft;  // VERY estimated, confirm with CAD
    constexpr auto kMaxHeight = 7_ft;  // VERY estimated, confirm with CAD
    constexpr auto kFirstStageLength = (kMaxHeight - kMinHeight)/3;    //extension length of stage 1
    constexpr auto kMassEffective = 33.86_kg;    // Approximated. Modeling 3-stage elevator as single stage
    //constexpr auto kMassE1 = 3_kg;    // Mass of the first section of the elevator extender
    //constexpr auto kMassE2 = 3_kg;    // Mass of the second section
    //constexpr auto kMassE3 = 3_kg;    // Mass of the third section

//Level Height
    
    constexpr units::length::centimeter_t goal_heights[] = {0_cm, 70_cm, 81_cm, 121_cm, 183_cm};

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

    // models the elevator
    frc::sim::ElevatorSim m_elevatorModel;

    // accesses internals of the talon FX objects to inject the simulated data
    ctre::phoenix6::sim::TalonFXSimState m_leadSim, m_followerSim;

    frc::sim::DIOSim m_bottomLimitSwitch;
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    frc::sim::DIOSim m_topLimitSwitch;
#endif
    
};

Elevator::Elevator() : m_leadMotor{ElevatorConstants::kLeadmotorID},
                       m_followerMotor{ElevatorConstants::kFollowermotorID},
                       m_reverseLimit{ElevatorConstants::kReverseLimitID},
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
                       m_forwardLimit{ElevatorConstants::kForwardLimitID},
#endif
                       m_sim_state{new ElevatorSim{*this}} {
    //Sets the follower motor
    using namespace ctre::phoenix6;
    //Sets and defines the Elevator motor PID config
    configs::TalonFXConfiguration m_ElevatorConfig;

    ctre::phoenix6::configs::HardwareLimitSwitchConfigs LimitConfig{};
    LimitConfig.ReverseLimitAutosetPositionEnable = true;
    LimitConfig.ReverseLimitAutosetPositionValue = 0_tr;

    m_followerMotor.SetControl(controls::Follower{ElevatorConstants::kLeadmotorID, false});
    m_ElevatorConfig.WithSlot0(configs::Slot0Configs{}
                        .WithKP(ElevatorConstants::kP)
                        .WithKI(ElevatorConstants::kI)
                        .WithKD(ElevatorConstants::kD)
                        .WithKG(ElevatorConstants::kG))
                    .WithHardwareLimitSwitch(LimitConfig);
    m_leadMotor.GetConfigurator().Apply(m_ElevatorConfig);

    // Ensures sensor is homed on boot.
    // The motor won't run while disabled, but if the limit switch is hit it should finish
    // Once the robot enables, if the limit switch hasn't been hit yet, this command
    // should stay scheduled so that the motor comes on and actually completes the 
    // homing procedure
    // Gravity should do this for us while disabled.
    frc2::CommandScheduler::GetInstance().Schedule(
        HomeEncoder()
            .IgnoringDisable(true)
            .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    );
}

void Elevator::Periodic() {
}

bool Elevator::IsAtPos(units::length::centimeter_t pos) {
    return (units::math::abs((pos - GetEndEffectorHeight())) <= (ElevatorConstants::kTolerance));
};

bool Elevator::IsAtLevel(Elevator::Level level) {
    return IsAtPos(ElevatorConstants::goal_heights[level] + ElevatorConstants::kMinHeight);
};

bool Elevator::isAtBottom() {
    return m_reverseLimit.Get();
};

bool Elevator::isAtTop() {
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    return m_forwardLimit.Get();
#else
    return GetEndEffectorHeight() >= ElevatorConstants::kMaxHeight;
#endif
};

units::centimeter_t Elevator::GetEndEffectorHeight() {
    const auto rotorTurns = m_leadMotor.GetPosition().GetValue();
    const auto sprocketTurns = rotorTurns / ElevatorConstants::kGearReduction;
    const auto firstStageHeight = sprocketTurns * ElevatorConstants::kSpoolCircum/1_tr;
    const auto thirdStageHeight = firstStageHeight * 3;
    return ElevatorConstants::kMinHeight + thirdStageHeight;
}

void Elevator::SetGoalHeight(const units::centimeter_t length) {    
    auto request = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
    
    const auto firstStageHeight = (length - ElevatorConstants::kMinHeight)/3;
    const auto sprocketTurns = firstStageHeight / ElevatorConstants::kSpoolCircum * 1_tr;
    const auto rotorTurns = sprocketTurns * ElevatorConstants::kGearReduction;
    
    m_leadMotor.SetControl(request
        .WithPosition(rotorTurns)
        .WithLimitReverseMotion(isAtBottom())
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
        .WithLimitForwardMotion(isAtTop())
#endif
    );
}

void Elevator::SetGoalHeight(Elevator::Level level) {
    SetGoalHeight(ElevatorConstants::goal_heights[level]);
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

frc2::CommandPtr Elevator::MoveUp(){
    return RunEnd ([this]{MotorMoveUp(); },
                   [this] {MotorStop(); });
}

frc2::CommandPtr Elevator::MoveDown(){
    return RunEnd ([this]{MotorMoveDown(); },
                   [this] {MotorStop(); });
}

frc2::CommandPtr Elevator::HomeEncoder() {
    return frc2::cmd::None();
}

frc2::CommandPtr Elevator::GoToLevel(Level goal) {
    return Run([this, goal] {SetGoalHeight(goal);})
        .Until([this, goal] {return IsAtLevel(goal);});
}

//***************************SIMULATION*****************************
ElevatorSim::ElevatorSim(Elevator& elevator):
    m_elevatorModel{
        frc::DCMotor::KrakenX60FOC(2),
        ElevatorConstants::kGearReduction,
        ElevatorConstants::kMassEffective,
        ElevatorConstants::kSpoolRadius,
        0_m,    // min height
        ElevatorConstants::kFirstStageLength,
        true,   // simulate gravity
        0_m     // starting height
    },
    m_leadSim{elevator.m_leadMotor},
    m_followerSim{elevator.m_followerMotor},
    m_bottomLimitSwitch{elevator.m_reverseLimit}
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    , m_topLimitSwitch{elevator.m_forwardLimit}
#endif
{
    // Randomize starting height to test limit switch
    static std::random_device rng{};
    std::uniform_real_distribution<double> start_height_dist{0, 1};
    const auto start_height = start_height_dist(rng)*ElevatorConstants::kFirstStageLength;
    m_elevatorModel.SetState(start_height, 0_mps);
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
        position * rotor_turns_per_elevator_height;

    const auto velocity = m_elevatorModel.GetVelocity();
    const units::turns_per_second_t rotor_velocity =
        velocity * rotor_turns_per_elevator_height;
    
    m_leadSim.SetRawRotorPosition(rotor_turns);
    m_leadSim.SetRotorVelocity(rotor_velocity);
    
    m_sim_state->m_bottomLimitSwitch.SetValue(m_elevatorModel.HasHitLowerLimit());
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    m_sim_state->m_topLimitSwitch.SetValue(m_elevatorModel.HasHitUpperLimit());
#endif

    // mechanically linked, though we should never read this value
    m_followerSim.SetRawRotorPosition(rotor_turns);
    m_followerSim.SetRotorVelocity(rotor_velocity);

    //Publishing data to NetworkTables
    frc::SmartDashboard::PutNumber("Elevator/Sim Position (m)", units::meter_t{position}.value());
}

Elevator::~Elevator() {}
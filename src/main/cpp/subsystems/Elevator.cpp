#include "subsystems/Elevator.h"

#include <frc/simulation/ElevatorSim.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/DIOSim.h>

#include <random>

namespace ElevatorConstants {
// Device Addresses
    int kLeadmotorID = 14;
    int kFollowermotorID = 15;
    int kBottomLimitSwitchID = 60;
    int kReverseLimitID = 1;
    int kForwardLimitID = 0;

// Physical Parameters
    constexpr auto kSprocketTeeth = 22;
    constexpr auto kDistancePerChainLink = 0.25_in;  // 25H "pitch" value
    constexpr auto kSprocketCircum = kSprocketTeeth * kDistancePerChainLink;
    constexpr auto kGearReduction = 8.5;  // Tentative, could change
    constexpr auto kMinHeight = 9.5_in;  // VERY estimated, confirm with CAD
    constexpr auto kMaxHeight = 89.5_in;  // VERY estimated, confirm with CAD
    constexpr auto kFirstStageLength = (kMaxHeight - kMinHeight)/3;    //extension length of stage 1
    constexpr auto kMassEffective = 33.86_kg;    // Approximated. Modeling 3-stage elevator as single stage
    //constexpr auto kMassE1 = 3_kg;    // Mass of the first section of the elevator extender
    //constexpr auto kMassE2 = 3_kg;    // Mass of the second section
    //constexpr auto kMassE3 = 3_kg;    // Mass of the third section

//Level Height
    

    constexpr units::length::centimeter_t kL1 = 90_cm;
    constexpr units::length::centimeter_t kL2 = 127_cm;
    constexpr units::length::centimeter_t kL3 = 150_cm;
    constexpr units::length::centimeter_t kL4 = 180_cm;
    constexpr units::length::centimeter_t kTolerance = 2_cm;

    constexpr units::length::centimeter_t goal_heights[] = {kMinHeight, kL1, kL2, kL3, kL4};


    constexpr units::length::centimeter_t softLimit = 85_in;

// Feedback/Feedforward Gains
    double kP = 1.0;
    double kI = 0.0;
    double kD = 0.0;
    constexpr auto kG = 0.351_V;
    double kS = 0.04;
    double kV = 0.11;
}

units::angle::turn_t lengthToRotorTurns(const units::centimeter_t height) {
  return (height - ElevatorConstants::kMinHeight) / (3 * ElevatorConstants::kSprocketCircum) 
        * ElevatorConstants::kGearReduction * 1_tr;
}

units::centimeter_t turnsToRobotHeight(units::angle::turn_t rotorTurns) {
    const auto sprocketTurns = rotorTurns / ElevatorConstants::kGearReduction;
    const auto firstStageHeight = sprocketTurns * ElevatorConstants::kSprocketCircum/1_tr;
    const auto thirdStageHeight = firstStageHeight * 3;
    return ElevatorConstants::kMinHeight + thirdStageHeight;
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

Elevator::Elevator() : m_leadMotor{ElevatorConstants::kLeadmotorID, "Drivebase"},
                       m_followerMotor{ElevatorConstants::kFollowermotorID, "Drivebase"},
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
                        .WithKG(ElevatorConstants::kG.value())
                        .WithKS(ElevatorConstants::kS)
                        .WithKV(ElevatorConstants::kV))
                    .WithHardwareLimitSwitch(LimitConfig);
    m_ElevatorConfig.WithSoftwareLimitSwitch(configs::SoftwareLimitSwitchConfigs{}
                        .WithForwardSoftLimitEnable(true)
                        .WithForwardSoftLimitThreshold(lengthToRotorTurns(ElevatorConstants::softLimit)));
    m_ElevatorConfig.WithMotorOutput(configs::MotorOutputConfigs{}
        .WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake)
        .WithInverted(signals::InvertedValue::Clockwise_Positive));
        
// set Motion Magic settings
auto& motionMagicConfigs = m_ElevatorConfig.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = units::angular_velocity::turns_per_second_t{65};
motionMagicConfigs.MotionMagicAcceleration = units::angular_acceleration::turns_per_second_squared_t{200}; 

m_leadMotor.GetConfigurator().Apply(m_ElevatorConfig);

    // Ensures sensor is homed on boot.
    // The motor won't run while disabled, but if the limit switch is hit it should finish
    // Once the robot enables, if the limit switch hasn't been hit yet, this command
    // should stay scheduled so that the motor comes on and actually completes the 
    // homing procedure
    // Gravity should do this for us while disabled.
    //frc2::CommandScheduler::GetInstance().Schedule(
    //    HomeEncoder()
    //        .IgnoringDisable(true)
    //        .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming)
    //);
}

void Elevator::Periodic() {
  frc::SmartDashboard::PutNumber(
      "EndEffector Height (cm)",
      ((units::inch_t)GetEndEffectorHeight()).value());

  frc::SmartDashboard::PutNumber(
      "Motor Voltage (V)", m_leadMotor.GetSupplyVoltage().GetValue().value());
}

bool Elevator::IsAtPos(units::length::centimeter_t pos) {
    return (units::math::abs((pos - GetEndEffectorHeight())) <= (ElevatorConstants::kTolerance));
};

bool Elevator::IsAtLevel(Elevator::Level level) {
    return IsAtPos(ElevatorConstants::goal_heights[level] + ElevatorConstants::kMinHeight);
};

bool Elevator::isAtBottom() {
    return !(m_reverseLimit.Get());
};

bool Elevator::isAtTop() {
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    return m_forwardLimit.Get();
#else
    return GetEndEffectorHeight() >= ElevatorConstants::kMaxHeight;
#endif
};


units::centimeter_t Elevator::GetEndEffectorHeight() {
    return turnsToRobotHeight(m_leadMotor.GetPosition().GetValue());
}


void Elevator::SetGoalHeight(const units::centimeter_t length) {    
    auto request = ctre::phoenix6::controls::MotionMagicVoltage{0_tr}.WithSlot(0);
    
    m_leadMotor.SetControl(request
        .WithPosition(lengthToRotorTurns(length))
        .WithLimitReverseMotion(isAtBottom())
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
        .WithLimitForwardMotion(isAtTop())
#endif
    );
}

void Elevator::SetGoalHeight(Elevator::Level level) {
    frc::SmartDashboard::PutNumber(
      "GoalLevel",
      ((units::inch_t)ElevatorConstants::goal_heights[level]).value());

    SetGoalHeight(ElevatorConstants::goal_heights[level]);
}

void Elevator::MotorMoveUp() {
    auto request = ctre::phoenix6::controls::VoltageOut{3_V};
    m_leadMotor.SetControl(request
        .WithLimitReverseMotion(isAtBottom())
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
        .WithLimitForwardMotion(isAtTop())
#endif
    );
};

void Elevator::MotorMoveDown() {
    auto request = ctre::phoenix6::controls::VoltageOut{-1.5_V};
    m_leadMotor.SetControl(request
        .WithLimitReverseMotion(isAtBottom())
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
        .WithLimitForwardMotion(isAtTop())
#endif
    );
};

void Elevator::MotorStop() {
    auto request = ctre::phoenix6::controls::VoltageOut{ElevatorConstants::kG};
    m_leadMotor.SetControl(request
        .WithLimitReverseMotion(isAtBottom())
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
        .WithLimitForwardMotion(isAtTop())
#endif
    );
};

frc2::CommandPtr Elevator::MoveUp(){
    return RunEnd ([this]{MotorMoveUp(); },
                   [this] {MotorStop(); });
}

frc2::CommandPtr Elevator::MoveDown(){
    return RunEnd ([this]{MotorMoveDown(); },
                   [this] {MotorStop(); });
}

//frc2::CommandPtr Elevator::HomeEncoder() {
//    return MoveDown().Until([this] {return isAtBottom();});
//}

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
        ElevatorConstants::kSprocketCircum/(2*std::numbers::pi),  // drum radius
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
    m_elevatorModel.SetInputVoltage(-m_leadSim.GetMotorVoltage());
    
    // Simulate the model over the next 20ms
    m_elevatorModel.Update(20_ms);

    // The motor turns kGearReduction times to turn the spool once
    // This raises the elevator one spool circumference up
    constexpr auto rotor_turns_per_elevator_height = 
        ElevatorConstants::kGearReduction * 1_tr / ElevatorConstants::kSprocketCircum;
    
    // Feed simulated outputs of model back into user program
    const auto position = m_elevatorModel.GetPosition();
    const units::turn_t rotor_turns =
        position * rotor_turns_per_elevator_height;

    const auto velocity = m_elevatorModel.GetVelocity();
    const units::turns_per_second_t rotor_velocity =
        velocity * rotor_turns_per_elevator_height;
    
    m_leadSim.SetRawRotorPosition(-rotor_turns);
    m_leadSim.SetRotorVelocity(-rotor_velocity);
    
    m_sim_state->m_bottomLimitSwitch.SetValue(!m_elevatorModel.HasHitLowerLimit());
#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    m_sim_state->m_topLimitSwitch.SetValue(!m_elevatorModel.HasHitUpperLimit());
#endif

    // mechanically linked, though we should never read this value
    m_followerSim.SetRawRotorPosition(-rotor_turns);
    m_followerSim.SetRotorVelocity(-rotor_velocity);

    //Publishing data to NetworkTables
    frc::SmartDashboard::PutNumber("Elevator/Sim Position (m)", units::meter_t{position}.value());
    frc::SmartDashboard::PutNumber("Elevator Voltage", 1);
}

Elevator::~Elevator() {}
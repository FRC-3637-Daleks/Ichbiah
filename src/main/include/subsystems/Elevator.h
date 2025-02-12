#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <memory>
#include <numbers>


// Forward Declaration
class ElevatorSim;

class Elevator: public frc2::SubsystemBase {
public:
    Elevator();
    ~Elevator();  // Need for reasons

    enum Level {INTAKE = 0, L1, L2, L3, L4, N};

    //Return bool on if its at the peram: pos
    bool IsAtPos(units::centimeter_t pos);
    bool IsAtLevel(Level level);

    //Sets the motor position (ends as soon as run)
    void SetGoalHeight(units::centimeter_t length);
    void SetGoalHeight(Level level);
    void SetTargetLevel(Level level);

    //Moves the motor up and down manually
    void MotorMoveUp();
    void MotorMoveDown();
    void MotorStop();

    // Gets the encoder position (used mostly by other functions)
    units::centimeter_t GetEndEffectorHeight();

    bool isAtTop();
    bool isAtBottom();

public:
    // Manual Override Commands
    frc2::CommandPtr MoveUp();
    frc2::CommandPtr MoveDown();

    // Default command, maintains the height the elevator was at when the command started
    frc2::CommandPtr HoldHeight();

    // Main command, commands elevator to goal and then ends when it reaches that point
    frc2::CommandPtr GoToLevel(Level goal);

    void Periodic();

private:
    ctre::phoenix6::hardware::TalonFX m_leadMotor;
    ctre::phoenix6::hardware::TalonFX m_followerMotor;

#ifdef ELEVATOR_TOP_LIMIT_SWITCH
    frc::DigitalInput m_forwardLimit;
#endif
    frc::DigitalInput m_reverseLimit;

    Level m_goalLevel;
 
private:  // simulation related members
    friend class ElevatorSim;
    std::unique_ptr<ElevatorSim> m_sim_state;
    void SimulationPeriodic() override;
};
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

    //Goes to the position, commnand only ends when 
    // destination is reached within the tolerance
    void GoToLevel(Level level);

    //Return bool on if its at the peram: pos
    bool IsAtPos(units::length::centimeter_t pos);
    bool IsAtLevel(Level level);

    //Sets the motor position (ends as soon as run)
    void SetMotorPosition(units::length::centimeter_t length);
    void SetMotorPosition(Level level);

    //Moves the motor up and down manually
    void MotorMoveUp();
    void MotorMoveDown();
    void MotorStop();

    // Gets the encoder position (used mostly by other functions)
    units::length::centimeter_t GetEncoderPosition();

    //Like the void cmds. but stops on button release
    frc2::CommandPtr WhileUp();
    frc2::CommandPtr WhileDown();
    bool getBottomBreakBeam();
    bool getTopBreakBeam();




    void Periodic();

private:
    ctre::phoenix6::hardware::TalonFX m_leadMotor;
    ctre::phoenix6::hardware::TalonFX m_followerMotor;

   //frc::DigitalInput m_forwardLimit;
    frc::DigitalInput m_reverseLimit;
 
    Level goalLevel;

private:  // simulation related members
    friend class ElevatorSim;
    std::unique_ptr<ElevatorSim> m_sim_state;
    void SimulationPeriodic() override;
};
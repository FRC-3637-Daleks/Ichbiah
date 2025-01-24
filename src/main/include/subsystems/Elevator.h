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

    void SetMotorPosition(units::angle::turn_t turns);

    void MotorMoveUp();
    void MotorMoveDown();
    void MotorStop();

    units::length::centimeter_t GetEncoderPosition();

    frc2::CommandPtr WhileUp();
    frc2::CommandPtr WhileDown();

private:
    ctre::phoenix6::hardware::TalonFX m_leadMotor;
    ctre::phoenix6::hardware::TalonFX m_followerMotor;

private:  // simulation related members
    friend class ElevatorSim;
    std::unique_ptr<ElevatorSim> m_sim_state;
    void SimulationPeriodic() override;
};
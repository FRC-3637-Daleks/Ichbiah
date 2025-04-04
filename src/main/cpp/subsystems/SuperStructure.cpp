#include "subsystems/SuperStructure.h"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

namespace SuperStructureConstants {
constexpr int kLaserID = 23;

constexpr grpl::LaserCanROI kLaserROI = {6, 0, 6, 16};

constexpr auto kBranchThreshold = 10_in;
}; // namespace SuperStructureConstants

class SuperStructureSim {
public:
  SuperStructureSim(SuperStructure &superstructure);

  grpl::MockLaserCan m_laser_sim;
};

SuperStructure::SuperStructure(Elevator &elevator, EndEffector &end_effector)
    : m_elevator(elevator), m_endeffector(end_effector),
      m_sim_state{new SuperStructureSim{*this}},
      m_laser{SuperStructureConstants::kLaserID} {

  // Set up LaserCANs
  m_laser.set_ranging_mode(grpl::LaserCanRangingMode::Short);
  m_laser.set_timing_budget(grpl::LaserCanTimingBudget::TB50ms);
  m_laser.set_roi(SuperStructureConstants::kLaserROI);
};

void SuperStructure::Periodic() { UpdateDashboard(); }

void SuperStructure::UpdateDashboard() {
  UpdateVisualization();
  frc::SmartDashboard::PutNumber(
      "SuperStructure/LaserCAN/Measurement (in)",
      units::inch_t{GetLaserCANMeasurement()}.value());
  frc::SmartDashboard::PutBoolean("Lined Up with Reef?", IsBranchInReach());
}

void SuperStructure::InitVisualization(frc::MechanismObject2d *elevator_root) {
  m_elevator.InitVisualization(elevator_root);
  m_endeffector.InitVisualization(m_elevator.GetElevatorLigament());
}

void SuperStructure::UpdateVisualization() {}

frc2::CommandPtr SuperStructure::prePlace(Elevator::Level level) {
  // Always intake first. Ensuring coral is reset to known position
  // Parallel empty command means the command runs indefinitely even after
  // completing the components
  return m_endeffector.Intake()
      .AndThen(m_elevator.GoToLevel(level).AlongWith(
          m_endeffector.EffectorContinue().AndThen(
              m_endeffector.MotorBackwardCommand().WithTimeout(0.2_s))))
      .AlongWith(Run([] {})); // keep parallel
};

frc2::CommandPtr SuperStructure::Intake() {
  // The command will run until the endeffector intakes
  return m_endeffector.Intake().DeadlineFor(
      m_elevator.GoToLevel(Elevator::INTAKE));
}

// Pre-requisit is having coral && being at the right
frc2::CommandPtr SuperStructure::Score(Elevator::Level level) {

  if (level == Elevator::L1) {
    // Move to the L1 prePlace state
    // Output at the L1 speed
    // while lifting elevator simultaneously to flip coral forth
    return prePlace(Elevator::L1)
        .Until([this] { return m_elevator.IsAtLevel(Elevator::L1); })
        .AndThen(m_endeffector.EffectorOutToL1().AlongWith(
            m_elevator.GoToLevel(Elevator::L2)));
  } else {
    // Ensure coral secured
    // Go to the level, wait to get there
    // Then output at the L2-4 speed
    // 2 second timeout just in case pressed with no coral inside
    return m_endeffector.Intake()
        .Unless([this] { return m_endeffector.IsOuterBreakBeamBroken(); })
        .AndThen(m_elevator.GoToLevel(level))
        .AndThen(m_endeffector.EffectorOut())
        .WithTimeout(2.0_s); // if left unchecked
  }
}

units::millimeter_t SuperStructure::GetLaserCANMeasurement() {
  auto measurement_opt = m_laser.get_measurement();
  if (measurement_opt.has_value() &&
      measurement_opt.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT)
    return units::millimeter_t{(double)measurement_opt.value().distance_mm};
  else
    return std::numeric_limits<units::millimeter_t>::max();
}

bool SuperStructure::IsBranchInReach() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return true;
  }

  bool var =
      GetLaserCANMeasurement() <= SuperStructureConstants::kBranchThreshold &&
      GetLaserCANMeasurement() > (units::length::millimeter_t)0.1;
  if (frc::SmartDashboard::GetString("Elevator/Target Level", "L1") == "L4") {
    frc::SmartDashboard::PutBoolean("BranchInReach?", var);
  } else {
    frc::SmartDashboard::PutBoolean("BranchInReach?", false);
  };
  return var;
}

bool SuperStructure::IsBranchInReachL23() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return true;
  }

  bool var = GetLaserCANMeasurement() <= 15_in &&
             GetLaserCANMeasurement() > (units::length::millimeter_t)0.1;
  // if (frc::SmartDashboard::GetString("Elevator/Target Level", "L1") == "L4")
  // {
  //   frc::SmartDashboard::PutBoolean("BranchInReach?", var);
  // } else {
  //   frc::SmartDashboard::PutBoolean("BranchInReach?", false);
  // };
  return var;
}

bool SuperStructure::ReadyToScore(Elevator::Level level) {
  if (m_elevator.IsAtLevel(level) && m_endeffector.HasCoral()) {
    if (level == Elevator::L4)
      return IsBranchInReach();
    else if (level == Elevator::L3 || level == Elevator::L3)
      return IsBranchInReachL23();
    else
      return true; // L1
  } else {
    return false; // not at the target height or lack coral
  }
}

SuperStructure::~SuperStructure() {}

/***************************  SIMULATION  **************************/

SuperStructureSim::SuperStructureSim(SuperStructure &superstructure) {}

void SuperStructure::SimulationPeriodic() {}
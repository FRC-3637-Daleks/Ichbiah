// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() { m_disabledCommand = m_container.FusePose(); }

void Robot::DriverStationConnected() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  std::string alliance =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
           ? "Red"
           : "Blue");

  // Needed for LEDs
  frc::SmartDashboard::PutString("Aliance Color", alliance);

  // Log the RIO states.
  frc::SmartDashboard::PutNumber(
      "RIO/Battery Voltage", frc::RobotController::GetBatteryVoltage() / 1_V);
  frc::SmartDashboard::PutNumber("RIO/CPU temp (C)",
                                 frc::RobotController::GetCPUTemp().value());
  frc::SmartDashboard::PutBoolean("RIO/Is Browned Out?",
                                  frc::RobotController::IsBrownedOut());

  // RIO 3.3V rail
  frc::SmartDashboard::PutBoolean("RIO/3.3V Rail/Is Enabled?",
                                  frc::RobotController::GetEnabled3V3());
  frc::SmartDashboard::PutNumber("RIO/3.3V Rail/Voltage",
                                 frc::RobotController::GetVoltage3V3());
  frc::SmartDashboard::PutNumber("RIO/3.3V Rail/Current",
                                 frc::RobotController::GetCurrent3V3());
  frc::SmartDashboard::PutNumber("RIO/3.3V Rail/Fault Count",
                                 frc::RobotController::GetFaultCount3V3());

  // RIO 5V rail
  frc::SmartDashboard::PutBoolean("RIO/5V Rail/Is Enabled?",
                                  frc::RobotController::GetEnabled5V());
  frc::SmartDashboard::PutNumber("RIO/5V Rail/Voltage",
                                 frc::RobotController::GetVoltage5V());
  frc::SmartDashboard::PutNumber("RIO/5V Rail/Current",
                                 frc::RobotController::GetCurrent5V());
  frc::SmartDashboard::PutNumber("RIO/5V Rail/Fault Count",
                                 frc::RobotController::GetFaultCount5V());

  // RIO 6V rail
  frc::SmartDashboard::PutBoolean("RIO/6V Rail/Is Enabled?",
                                  frc::RobotController::GetEnabled6V());
  frc::SmartDashboard::PutNumber("RIO/6V Rail/Voltage",
                                 frc::RobotController::GetVoltage6V());
  frc::SmartDashboard::PutNumber("RIO/6V Rail/Current",
                                 frc::RobotController::GetCurrent6V());
  frc::SmartDashboard::PutNumber("RIO/6V Rail/Fault Count",
                                 frc::RobotController::GetFaultCount6V());

  auto can_status = frc::RobotController::GetCANStatus();

  frc::SmartDashboard::PutNumber("CAN Bus/Percent CAN Utilization",
                                 can_status.percentBusUtilization * 100.f);
  frc::SmartDashboard::PutNumber("CAN Bus/Bus Off Count",
                                 can_status.busOffCount);
  frc::SmartDashboard::PutNumber("CAN Bus/Recieve Error Count",
                                 can_status.receiveErrorCount);
  frc::SmartDashboard::PutNumber("CAN Bus/Transmit Error Count",
                                 can_status.transmitErrorCount);
  frc::SmartDashboard::PutNumber("CAN Bus/TX Full Count",
                                 can_status.txFullCount);

  frc::SmartDashboard::PutNumber("DS Match Time",
                                 frc::DriverStation::GetMatchTime().value());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  if (m_disabledCommand)
    m_disabledCommand->Schedule();
}

void Robot::DisabledExit() {
  if (m_disabledCommand)
    m_disabledCommand->Cancel();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  frc::Shuffleboard::SelectTab("Autonomous");

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand.has_value()) {
    m_autonomousCommand.value()->Schedule();
  }

  if (IsSimulation()) {
    m_container.m_endeffector.SimulatePreloadCoral();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { frc::Shuffleboard::SelectTab("Teleoperated"); }

/**
 * This function is called periodically
 *  during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {
  constexpr auto field_width = 26_ft + 5_in;
  constexpr auto field_length = 57_ft + 6.875_in;
  constexpr frc::Translation2d corners[] = {{0_m, 0_m},
                                            {0_m, field_width},
                                            {field_length, 0_m},
                                            {field_length, field_width}};
  constexpr frc::Translation2d robot_intake{0_in, 15_in};

  const auto robot_pose = m_container.m_swerve.GetPose();
  const auto intake_pose = robot_pose.TransformBy({robot_intake, 0_deg});

  for (const auto corner : corners) {
    if (intake_pose.Translation().Distance(corner) < 4_ft) {
      // TODO: send it to intake first
      m_container.m_endeffector.SimulateNewCoral();
    }
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/JoystickButton.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <functional>
#include <numbers>

#include "PathFollower.h"
#include "subsystems/Climb.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/LEDSubsystem.h"
#include "subsystems/OperatorInterface.h"
#include "subsystems/ROSBridge.h"
#include "subsystems/ReefAssist.h"
#include "subsystems/SuperStructure.h"
#include "subsystems/Vision.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetDisabledCommand();
  frc2::Command *GetAutonomousCommand();

public:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems are defined here...

  Drivetrain m_swerve;
  ROSBridge m_ros;
  OperatorInterface m_oi;

  /* Pass elevator and end effector by reference to super structure
   * Allows us to directly control or query elevator and endeffector for
   * diagnostics while allowing super structure to define high level controls in
   * another file
   */
  Elevator m_elevator;
  EndEffector m_endeffector;
  SuperStructure m_superStructure{m_elevator, m_endeffector};
  Climb m_climb;
  LEDSubsystem m_ledSubsystem;

  Vision m_vision;

  bool m_isRed;
  frc::Pose2d reefPose;

  std::function<bool()> m_updateIsRed = [this]() -> bool {
    return frc::DriverStation::GetAlliance() ==
           frc::DriverStation::Alliance::kRed;
  };

  frc::Mechanism2d m_mech{4, 8}; // scaled to feet
  frc::SendableChooser<frc2::Command *> m_chooser;

  frc2::CommandPtr threel4auto{frc2::cmd::None()};
  frc2::CommandPtr threel4autoprocessor{frc2::cmd::None()};
  frc2::CommandPtr onel4startmidauto{frc2::cmd::None()};
  frc2::CommandPtr drivethingy{frc2::cmd::None()};

public:
  void ConfigureBindings();
  void ConfigureDashboard();
  void ConfigureAuto();
  void ConfigureContinuous();

  // Command which fuses in the best guess for the robot pose from sensors
  // Can run while disabled, at the start of auton, when aligning to the reef,
  // or always Making it its own command gives programmers finer control over
  // when the pose estimate is taken over.
  frc2::CommandPtr FusePose();

public:
  bool IsRed();
};
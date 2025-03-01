#include "subsystems/OperatorInterface.h"

#include <frc/RobotBase.h>
#include <frc/GenericHID.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/button/Trigger.h>
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>

//FEALS WRONG ERIC PLEASE FIX
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <units/math.h>

    namespace OperatorConstants {
        constexpr int kCopilotControllerPort = 1;
        constexpr int kSwerveControllerPort = 0;
    
        constexpr double kStrafeDeadband = 0.08;
        constexpr double kRotDeadband = .16;
        constexpr double kClimbDeadband = 0.08;
        constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;
    
        constexpr auto kMaxTeleopSpeed = 15.7_fps;
        constexpr auto kMaxTeleopTurnSpeed = 2.5 * std::numbers::pi * 1_rad_per_s;
}
    OperatorInterface::OperatorInterface() :
        m_swerveController{OperatorConstants::kSwerveControllerPort},  
        m_copilotController{OperatorConstants::kCopilotControllerPort}  {
    }

double OperatorInterface::throttle() {
  double input = m_swerveController.GetHID().GetRightTriggerAxis();
  double ret = ((-input +1))/2;
  return ret;
}
units::meters_per_second_t OperatorInterface::fwd() {
  auto input = frc::ApplyDeadband(
    m_swerveController.GetHID().GetLeftY(),
    OperatorConstants::kStrafeDeadband);
  auto squaredInput = input * std::abs(input);
  auto alliance_flip = IsRed()? -1:1;
  return OperatorConstants::kMaxTeleopSpeed 
    * squaredInput
    * alliance_flip
    * throttle();
}

units::meters_per_second_t OperatorInterface::strafe() {
  auto input = frc::ApplyDeadband(
    m_swerveController.GetHID().GetLeftX(),
    OperatorConstants::kStrafeDeadband);
  auto squaredInput = input * std::abs(input);
  auto alliance_flip = IsRed()? -1:1;
  return OperatorConstants::kMaxTeleopSpeed 
    * squaredInput
    * alliance_flip
    * throttle();
}

units::revolutions_per_minute_t OperatorInterface::rot() {
  auto input = frc::ApplyDeadband(
    -m_swerveController.GetHID().GetRightY(),
    OperatorConstants::kRotDeadband);
  auto squaredInput = input * std::abs(input);
  return OperatorConstants::kMaxTeleopTurnSpeed
    * squaredInput
    * throttle();
  };

//FEELS WRONG PLEASE ERIC FIX
frc2::CommandPtr OperatorInterface::RumbleDriver(units::time::second_t time, double intensity) {
  return frc2::cmd::Run([this, time, intensity] {
    if (RumbleCount < time.value()/0.020) {
      m_swerveController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, intensity); 
      m_swerveController.SetRumble(frc::GenericHID::RumbleType::kRightRumble, intensity); 
    } else {
      m_swerveController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, intensity); 
      m_swerveController.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, intensity);
    }
    RumbleCount++;
    });
  
}

frc2::CommandPtr OperatorInterface::SetRumbleCountToZero() {
  return frc2::cmd::Run([this] {
    RumbleCount = 0;
  });
}

bool OperatorInterface::IsRed()
{
 auto m_isRed = (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed);
  return m_isRed;
}

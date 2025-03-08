#pragma once

#include "Piston.h"

#include <frc/Compressor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/DigitalInput.h>

class Climb : public frc2::SubsystemBase {
public:
  Climb();
  frc2::CommandPtr ExtendClimb();
  frc2::CommandPtr RetractClimb();
  frc2::CommandPtr ToggleClimbCommand();

  void Periodic() override;

private:
  Piston m_dualPistons;
  frc::Compressor m_compressor;

  frc::DigitalInput m_limSwitch;
};
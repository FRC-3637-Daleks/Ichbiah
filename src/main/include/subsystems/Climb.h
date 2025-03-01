#pragma once

#include "Piston.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

class Climb : public frc2::SubsystemBase {
public:
  Climb();
  frc2::CommandPtr ExtendClimb();
  frc2::CommandPtr RetractClimb();

  void Periodic() override;

private:
  Piston m_dualPistons;
};
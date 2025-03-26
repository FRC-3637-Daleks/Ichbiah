#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <units/time.h>

class Piston : public frc2::SubsystemBase {

public:
  enum State { Extending, Extended, Retracting, Retracted };

  Piston(int fChannel, int rChannel, units::second_t delay);
  frc2::CommandPtr Extend();
  frc2::CommandPtr Retract();
  frc2::CommandPtr Off();
  State getState();

private:
  frc::DoubleSolenoid m_solenoid;
  units::second_t stroke_delay;

  State m_state;
};
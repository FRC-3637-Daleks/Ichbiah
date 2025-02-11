#pragma once

#include "Piston.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

class Climb : public frc2::SubsystemBase {
    public:
        Climb();
        frc2::CommandPtr StartClimb();

    private:
        Piston m_pistonLeft;
        Piston m_pistonRight;
};
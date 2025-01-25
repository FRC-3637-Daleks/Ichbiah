#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

enum state {
            Extending
            Extended
            Retracting
            Retracted
        };

class Piston {

    public:
        Piston(frc::PneumaticsModuleType mType, int fChannel, int rChannel, units::second_t delay);
        frc2::CommandPtr Extend();
        frc2::CommandPtr Retract();
        frc2::CommandPtr Off();
        bool isExtended();
        bool isRetracted();

    private:
        frc::DoubleSolenoid m_solenoid;
        const units::second_t stroke_delay;

        enum state m_state
};
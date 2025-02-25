#pragma once

#include <frc2/command/SubsystemBase.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
    LEDSubsystem();
    void Periodic() override;

    private:
    static constexpr int kPort = 0;
    static constexpr int kLength = 250;
    //frc::AddressableLED m_led{kPort};
};
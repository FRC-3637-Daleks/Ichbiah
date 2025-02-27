#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
    LEDSubsystem();
    void Periodic() override;

    private:
    static constexpr int kPort = 0;

    static constexpr int baseFrontLength = 10;
    static constexpr int baseRightLength = 10;
    static constexpr int baseBackLength = 10;
    static constexpr int baseLeftLength = 10;
    static constexpr int elevLeftLength = 10;
    static constexpr int elevRightLength = 10;

    static constexpr int kLength = baseFrontLength + baseRightLength + baseBackLength + baseLeftLength + elevLeftLength + elevRightLength;

    frc::AddressableLED m_led{kPort};
    std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;

    std::span<frc::AddressableLED::LEDData, baseFrontLength> baseFront{m_ledBuffer.begin(), baseFrontLength};
    std::span<frc::AddressableLED::LEDData, baseRightLength> baseRight{baseFront.end(), baseRightLength};
    std::span<frc::AddressableLED::LEDData, baseBackLength> baseBack{baseRight.end(), baseBackLength};
    std::span<frc::AddressableLED::LEDData, baseLeftLength> baseLeft{baseBack.end(), baseLeftLength};
    std::span<frc::AddressableLED::LEDData, elevLeftLength> elevLeft{baseLeft.end(), elevLeftLength};
    std::span<frc::AddressableLED::LEDData, elevRightLength> elevRight{elevLeft.end(), elevRightLength};

    static constexpr int numSpans = 6;
    std::array<std::span<frc::AddressableLED::LEDData>, numSpans> m_ledSegments;
    std::array<int, numSpans> m_ledSegmentLengths{10, 10, 10, 10, 10, 10};

};
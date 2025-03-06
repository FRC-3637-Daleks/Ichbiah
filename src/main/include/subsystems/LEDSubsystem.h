#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
    LEDSubsystem();
    void Periodic() override;

    private:
    static constexpr int kPort{0};
    static constexpr int kNumSpans{3};
    static constexpr int kLength{150};
    std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer{};
    std::array<uint32_t, kNumSpans> m_ledSegmentLengths{47, 26 ,47};

  enum class LEDSEGMENT {
        ElevLeft,
        ElevTop,
        ElevRight,
    };
  
    enum class LEDSTATE{
        Default,
    };

    LEDSTATE m_currState {LEDSTATE::Default};

    std::vector<frc::LEDPattern > m_spanPatterns{};

    //LED Strip has 60 LEDs / meter
    units::meter_t kLedSpacing{1 / 60.0};

    frc::AddressableLED m_led{kPort};

    std::array<std::span<frc::AddressableLED::LEDData>,kNumSpans> m_ledSegments;

    void setState(LEDSTATE state);
    std::span<frc::AddressableLED::LEDData> getSegment(int segment);
    std::span<frc::AddressableLED::LEDData> getSegment(LEDSEGMENT segment);
    frc::LEDPattern& getSpanPattern(LEDSEGMENT segment);
    void setAllSpanPatterns(const frc::LEDPattern& pattern);
};
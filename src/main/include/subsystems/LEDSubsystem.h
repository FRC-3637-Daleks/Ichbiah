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
    static constexpr int kNumSpans{6};
    std::array<frc::AddressableLED::LEDData, 60> m_ledBuffer{};
    std::array<uint32_t, kNumSpans> m_ledSegmentLengths{10, 10, 10, 10, 10, 10};

  enum class LEDSEGMENT {
        Front,
        Right,
        Back,
        Left,
        ElevLeft,
        ElevRight,
    };
  
    enum class LEDSTATE{
        BaseNoCoral,
        CoralEntersRobot,
        CoralEntersIntake,
        BaseWithCoral,
        ElevatorLevel_1,
        ElevatorLevel_2,
        ElevatorLevel_3,
        ElevatorLevel_4,
    };

    LEDSTATE m_currState {LEDSTATE::BaseNoCoral};

    //LED Strip has 60 LEDs / meter
    units::meter_t kLedSpacing{1 / 60.0};

    frc::AddressableLED m_led{kPort};

    std::array<std::span<frc::AddressableLED::LEDData>,kNumSpans> m_ledSegments;

    void setState(LEDSTATE state);
    std::span<frc::AddressableLED::LEDData> getSegment(LEDSEGMENT segment);
};
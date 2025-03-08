#pragma once

#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/SubsystemBase.h>

namespace SDCONST {
const std::string isRed{"IsRedAlliance"};
const std::string coralInIntake{"EndEffector/has coral?"};
const std::string cageIntaked("Climb/cage intaked?");
} // namespace SDCONST

class LEDSubsystem : public frc2::SubsystemBase {
public:
  LEDSubsystem();
  void Periodic() override;

private:
  static constexpr int kPort{0};
  static constexpr int kNumSpans{1};
  static constexpr int kLength{128};
  std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer{};
  std::array<uint32_t, kNumSpans> m_ledSegmentLengths{128};

  enum class LEDSEGMENT {
    ElevLeft,
    ElevTop,
    ElevRight,
  };

  enum class LEDSTATE {
    Default,
    CoralInIntake1,
    CoralInIntake2,
    CageIntaked1,
    CageIntaked2,
  };

  LEDSTATE m_currState{LEDSTATE::Default};

  std::vector<frc::LEDPattern> m_spanPatterns{};

  // LED Strip has 60 LEDs / meter
  units::meter_t kLedSpacing{1 / 60.0};

  frc::AddressableLED m_led{kPort};

  std::array<std::span<frc::AddressableLED::LEDData>, kNumSpans> m_ledSegments;

  double countDown = 0.0;
  std::chrono::time_point<std::chrono::system_clock> previousFrameTime{
      std::chrono::system_clock::now()};

  void setState(LEDSTATE state);
  std::span<frc::AddressableLED::LEDData> getSegment(int segment);
  std::span<frc::AddressableLED::LEDData> getSegment(LEDSEGMENT segment);
  frc::LEDPattern &getSpanPattern(LEDSEGMENT segment);
  void setAllSpanPatterns(const frc::LEDPattern &pattern);
};
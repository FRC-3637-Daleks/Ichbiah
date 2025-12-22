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
  double countDown2 = 0.0;
  std::chrono::time_point<std::chrono::system_clock> previousFrameTime{
      std::chrono::system_clock::now()};

  void setState(LEDSTATE state);
  std::span<frc::AddressableLED::LEDData> getSegment(int segment);
  std::span<frc::AddressableLED::LEDData> getSegment(LEDSEGMENT segment);
  frc::LEDPattern &getSpanPattern(LEDSEGMENT segment);
  void setAllSpanPatterns(const frc::LEDPattern &pattern);
};

#include "subsystems/LEDSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

#include <frc/DriverStation.h>

LEDSubsystem::LEDSubsystem() {
  // Setup LED Segments
  frc::LEDPattern m_black = frc::LEDPattern::Solid(frc::Color::kBlack);

  int spanOffset = 0;
  for (int i = 0; i < kNumSpans; i++) {
    m_ledSegments[i] = std::span<frc::AddressableLED::LEDData>{
        m_ledBuffer.begin() + spanOffset, m_ledSegmentLengths[i]};
    spanOffset += m_ledSegmentLengths[i];
    m_spanPatterns.push_back(m_black);
  }

  // Setup AddressibleLED Object
  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();

  m_black.ApplyTo(m_ledBuffer);
  m_led.SetData(m_ledBuffer);

  setState(LEDSTATE::Default);
}

void LEDSubsystem::Periodic() {
  //   const bool hasCoral =
  //       frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false);
  switch (m_currState) {
  case LEDSTATE::Default:
    if (frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
      setState(LEDSTATE::CoralInIntake1);
      countDown = 500;
    }
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    }
    // if (frc::SmartDashboard::GetBoolean("BranchInReach?", false)) {
    //   setState(LEDSTATE::CageIntaked1);
    //   countDown2 = 500;
    // }
    // if (!frc::SmartDashboard::GetBoolean("BranchInReach?", false)) {
    //   setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color::kRed));
    // }
    break;
  case LEDSTATE::CoralInIntake1:
    if (countDown <= 0.0) {
      countDown = 250;
      setState(LEDSTATE::CoralInIntake2);
    }
    if (!frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
      setState(LEDSTATE::Default);
    }
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    }
    break;
  case LEDSTATE::CoralInIntake2:
    if (countDown <= 0.0) {
      countDown = 250;
      setState(LEDSTATE::CoralInIntake1);
    }
    if (!frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
      setState(LEDSTATE::Default);
    }
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    }
    break;
  case LEDSTATE::CageIntaked1:
    if (countDown <= 0.0) {
      countDown = 125;
      setState(LEDSTATE::CageIntaked2);
    }
    if (countDown2 <= 0.0) {
      setState(LEDSTATE::Default);
    }
    break;
  case LEDSTATE::CageIntaked2:
    if (countDown <= 0.0) {
      countDown = 125;
      setState(LEDSTATE::CageIntaked1);
    }
    if (countDown2 <= 0.0) {
      setState(LEDSTATE::Default);
    }
    break;
  }

  setState(m_currState);

  for (int i = 0; i < kNumSpans; i++) {
    m_spanPatterns[i].ApplyTo(getSegment(i));
  }

  m_led.SetData(m_ledBuffer);

  if (countDown > 0.0) {
    countDown -=
        std::chrono::duration<double, std::milli>{
            std::chrono::system_clock::now() - previousFrameTime}
            .count();
  }
  if (countDown2 > 0.0 &&
      !frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
    countDown2 -=
        std::chrono::duration<double, std::milli>{
            std::chrono::system_clock::now() - previousFrameTime}
            .count();
  }
  previousFrameTime = std::chrono::system_clock::now();

  if (frc::SmartDashboard::GetBoolean("BranchInReach?", false)) {
    setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color::kRed));
  }
}

void LEDSubsystem::setState(LEDSTATE state) {
  switch (state) {
  case LEDSTATE::CoralInIntake1:
    setAllSpanPatterns(frc::LEDPattern::Gradient(
                           frc::LEDPattern::GradientType::kContinuous,
                           std::array<frc::Color, 2>{frc::Color{127, 127, 0},
                                                     frc::Color{0, 255, 0}})
                           .ScrollAtAbsoluteSpeed(1_mps, kLedSpacing));
    break;
  case LEDSTATE::CageIntaked1:
    setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color{0.0, 1.0, 1.0}));
    break;
  case LEDSTATE::CageIntaked2:
    setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color{0.2, 0.0, 0.2}));
    break;
  default:
    const bool isRed =
        frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    if (isRed) {
      setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color::kRed));
    } else {
      setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color::kBlue));
    }
    break;
  }

  m_currState = state;
}

std::span<frc::AddressableLED::LEDData> LEDSubsystem::getSegment(int segment) {
  return m_ledSegments[segment];
}

std::span<frc::AddressableLED::LEDData>
LEDSubsystem::getSegment(LEDSEGMENT segment) {
  return getSegment(static_cast<int>(segment));
}

frc::LEDPattern &LEDSubsystem::getSpanPattern(LEDSEGMENT segment) {
  return m_spanPatterns[static_cast<int>(segment)];
}

void LEDSubsystem::setAllSpanPatterns(const frc::LEDPattern &pattern) {
  for (int i = 0; i < kNumSpans; i++) {
    m_spanPatterns[i] = pattern;
  }
}

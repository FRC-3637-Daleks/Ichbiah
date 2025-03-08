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
      setState(LEDSTATE::CoralInIntake);
    }
    break;
  case LEDSTATE::CoralInIntake:
    if (!frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
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
  previousFrameTime = std::chrono::system_clock::now();
}

void LEDSubsystem::setState(LEDSTATE state) {
  switch (state) {
  case LEDSTATE::CoralInIntake:
    setAllSpanPatterns(frc::LEDPattern::Gradient(
                           frc::LEDPattern::GradientType::kContinuous,
                           std::array<frc::Color, 2>{frc::Color{127, 127, 0},
                                                     frc::Color{0, 255, 0}})
                           .ScrollAtAbsoluteSpeed(1_mps, kLedSpacing));
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

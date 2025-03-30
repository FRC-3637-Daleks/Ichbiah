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
  switch (m_currState) {
  case LEDSTATE::Default:
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    } else if (frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
      if (frc::SmartDashboard::GetBoolean(SDCONST::branchFound, false)) {
        setState(LEDSTATE::BranchFound);
      } else {
        setState(LEDSTATE::CoralInIntake);
      }
    }
    break;

  case LEDSTATE::CoralInIntake:
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    } else if (!frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake,
                                                false)) {
      setState(LEDSTATE::Default);
    } else if (frc::SmartDashboard::GetBoolean(SDCONST::branchFound, false)) {
      setState(LEDSTATE::BranchFound);
    }
    break;

  case LEDSTATE::BranchFound:
    if (frc::SmartDashboard::GetBoolean(SDCONST::cageIntaked, false)) {
      setState(LEDSTATE::CageIntaked1);
      countDown2 = 3000;
    } else if (!frc::SmartDashboard::GetBoolean(SDCONST::branchFound, false)) {
      if (frc::SmartDashboard::GetBoolean(SDCONST::coralInIntake, false)) {
        setState(LEDSTATE::CoralInIntake);
      } else {
        setState(LEDSTATE::Default);
      }
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
}

void LEDSubsystem::setState(LEDSTATE state) {
  switch (state) {
  case LEDSTATE::CoralInIntake:
    setAllSpanPatterns(frc::LEDPattern::Solid(frc::Color{1.0, 0.0, 0.0}));
    break;

  case LEDSTATE::BranchFound:
    setAllSpanPatterns(frc::LEDPattern::Rainbow(255, 255).ScrollAtAbsoluteSpeed(
        1_mps, kLedSpacing));
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

#include "subsystems/LEDSubsystem.h"

LEDSubsystem::LEDSubsystem()
{
    m_ledSegments[0] = std::span<frc::AddressableLED::LEDData>{m_ledBuffer.begin(), m_ledSegmentLengths[0]};

    auto currIter{m_ledSegments[0].end()};
    int totalLeds{m_ledSegmentLengths[0]};

    for (int i = 1; i < numSpans; i++) {
        m_ledSegments[i] = std::span<frc::AddressableLED::LEDData>{currIter, m_ledSegmentLengths[i]};
        currIter = m_ledSegments[i].end();
        totalLeds += m_ledSegmentLengths[i];
    }
    
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

    frc::LEDPattern red = frc::LEDPattern::Solid(frc::Color::kRed);
    frc::LEDPattern orange = frc::LEDPattern::Solid(frc::Color::kOrange);
    frc::LEDPattern yellow = frc::LEDPattern::Solid(frc::Color::kYellow);
    frc::LEDPattern green = frc::LEDPattern::Solid(frc::Color::kGreen);
    frc::LEDPattern blue = frc::LEDPattern::Solid(frc::Color::kBlue);
    frc::LEDPattern purple = frc::LEDPattern::Solid(frc::Color::kPurple);

    frc::LEDPattern white = frc::LEDPattern::Solid(frc::Color::kWhite);
    frc::LEDPattern black = frc::LEDPattern::Solid(frc::Color{0,0,0});

    red.ApplyTo(baseFront);
    orange.ApplyTo(baseRight);
    yellow.ApplyTo(baseBack);
    green.ApplyTo(baseLeft);
    blue.ApplyTo(elevLeft);
    purple.ApplyTo(elevRight);

    // red.ApplyTo(m_ledBuffer);
    // black.ApplyTo(m_ledBuffer);
    // white.ApplyTo(m_ledBuffer);
}

void LEDSubsystem::Periodic()
{
    m_led.SetData(m_ledBuffer);
}

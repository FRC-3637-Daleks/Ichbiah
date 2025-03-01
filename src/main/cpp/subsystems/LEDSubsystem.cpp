#include "subsystems/LEDSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

LEDSubsystem::LEDSubsystem()
{
    // Get total LEDs
    int  totalLEDs = 0;
    for(int i = 0; i < kNumSpans; i++){
        totalLEDs += m_ledSegmentLengths[i];
    }

    // Setup LED Segments
    m_ledSegments[0] = std::span<frc::AddressableLED::LEDData>{m_ledBuffer.begin(), m_ledSegmentLengths[0]};

    auto currIter{m_ledSegments[0].end()};
    for (int i = 1; i < kNumSpans; i++) {
        m_ledSegments[i] = std::span<frc::AddressableLED::LEDData>{currIter, m_ledSegmentLengths[i]};
        currIter = m_ledSegments[i].end();
    }

    // Setup AddressibleLED Object 
    m_led.SetLength(totalLEDs);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
 }

void LEDSubsystem::Periodic()
{
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);
    m_scrollingRainbow.ApplyTo(m_ledBuffer);
   
    m_led.SetData(m_ledBuffer);


}

void LEDSubsystem::setState(LEDSTATE state)
{
    switch(state){
        case LEDSTATE::BaseNoCoral: break;
        case LEDSTATE::CoralEntersRobot: break;
        case LEDSTATE::CoralEntersIntake: break;
        case LEDSTATE::BaseWithCoral: break;
        case LEDSTATE::ElevatorLevel_1: break;
        case LEDSTATE::ElevatorLevel_2: break;
        case LEDSTATE::ElevatorLevel_3: break;
        case LEDSTATE::ElevatorLevel_4: break;
    }
}

std::span<frc::AddressableLED::LEDData> LEDSubsystem::getSegment(LEDSEGMENT segment)
{
    return m_ledSegments[static_cast<uint32_t>(segment)];
}

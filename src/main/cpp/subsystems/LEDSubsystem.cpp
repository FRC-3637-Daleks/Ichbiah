#include "subsystems/LEDSubsystem.h"

LEDSubsystem::LEDSubsystem()
{
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

    frc::LEDPattern red = frc::LEDPattern::Solid(frc::Color::kRed);
    frc::LEDPattern orange = frc::LEDPattern::Solid(frc::Color::kOrange);
    frc::LEDPattern yellow = frc::LEDPattern::Solid(frc::Color::kYellow);
    frc::LEDPattern green = frc::LEDPattern::Solid(frc::Color::kGreen);
    frc::LEDPattern blue = frc::LEDPattern::Solid(frc::Color::kBlue);
    frc::LEDPattern purple = frc::LEDPattern::Solid(frc::Color::kPurple);

    red.ApplyTo(baseFront);
    orange.ApplyTo(baseRight);
    yellow.ApplyTo(baseBack);
    green.ApplyTo(baseLeft);
    blue.ApplyTo(elevLeft);
    purple.ApplyTo(elevRight);
}

void LEDSubsystem::Periodic()
{
    m_led.SetData(m_ledBuffer);
}

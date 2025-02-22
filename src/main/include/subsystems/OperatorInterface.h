#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>


#include <units/math.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

class OperatorInterface {
    
public: 
    OperatorInterface();
    double throttle();
    units::meters_per_second_t strafe();
    units::meters_per_second_t fwd();
    units::revolutions_per_minute_t rot();
    frc2::Trigger zeroHeadingTrigger{
        [this]() ->bool { return m_swerveController.Button(12).Get(); }};
    frc2::Trigger ElevatorL1Trigger{
        [this]() ->bool { return m_swerveController.Button(11).Get(); }};
    frc2::Trigger ElevatorL2Trigger{
        [this]() ->bool { return m_swerveController.Button(9).Get(); }};
    frc2::Trigger ElevatorL3Trigger{
        [this]() ->bool { return m_swerveController.Button(7).Get(); }};
    frc2::Trigger ElevatorL4Trigger{
        [this]() ->bool { return m_swerveController.Button(2).Get(); }};
    frc2::Trigger EndEffectorInTrigger{
        [this]() ->bool { return m_swerveController.Button(3).Get(); }};
    frc2::Trigger EndEffectorOutTrigger{
        [this]() ->bool { return m_swerveController.Button(5).Get(); }};
    frc2::Trigger ElevatorUpTrigger{
        [this]() ->bool { return m_swerveController.Button(4).Get(); }};
    frc2::Trigger ElevatorDownTrigger{
        [this]() ->bool { return m_swerveController.Button(6).Get(); }};
    frc2::Trigger ClimbUpTrigger{
        [this]() ->bool { return m_swerveController.Button(8).Get(); }};
    frc2::Trigger ClimbDownTrigger{
        [this]() ->bool { return m_swerveController.Button(10).Get(); }};
    frc2::Trigger FollowPathTrigger{
        [this]() ->bool { return m_swerveController.POVDown().Get(); }};
    frc2::Trigger DriveToPoseTrigger{
        [this]() ->bool { return m_swerveController.POVDown().Get(); }};

private:
    frc2::CommandJoystick m_swerveController;
    //frc2::CommandXboxController m_copilotController;
    bool IsRed();
};
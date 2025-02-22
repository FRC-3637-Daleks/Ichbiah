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
    frc2::Trigger ElevatorL1Trigger = m_swerveController.Button(11);
    frc2::Trigger ElevatorL2Trigger = m_swerveController.Button(9);
    frc2::Trigger ElevatorL3Trigger = m_swerveController.Button(7);
    frc2::Trigger ElevatorL4Trigger = m_swerveController.Button(2);
    frc2::Trigger ElevatorUpTrigger = m_swerveController.Button(4);
    frc2::Trigger ElevatorDownTrigger = m_swerveController.Button(6);
    frc2::Trigger EndEffectorInTrigger= m_swerveController.Button(3);
    frc2::Trigger EndEffectorOutTrigger = m_swerveController.Button(5);
    frc2::Trigger ClimbUpTrigger = m_swerveController.Button(8);
    frc2::Trigger ClimbDownTrigger = m_swerveController.Button(10);
    frc2::Trigger ElevatorIntakeTrigger = m_swerveController.Button(1);
    frc2::Trigger FollowPathTrigger = m_swerveController.POVDown();
    frc2::Trigger DriveToPoseTrigger = m_swerveController.POVDown();
    frc2::Trigger zeroHeadingTrigger = m_swerveController.Button(12);

private:
    frc2::CommandJoystick m_swerveController;
    //frc2::CommandXboxController m_copilotController;
    bool IsRed();
};
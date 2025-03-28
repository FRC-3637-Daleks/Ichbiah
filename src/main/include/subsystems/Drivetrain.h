#pragma once

#include <studica/AHRS.h>

#include <choreo/Choreo.h>

#include <frc/PowerDistribution.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>

#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include <memory>
#include <numbers>
#include <utility>

#include "DrivetrainInterfaceHelper.h"
#include "swerve/OdometryThread.h"
#include "swerve/SwerveModule.h"

// Forward Declaration
class DrivetrainSimulation;

/**
 * The Drivetrain subsystem contains four swerve modules and a gyroscope. The
 * Drivetrain can be driven at a certain speed represented as a 2-dimensional
 * translation on the field. This translation can be commanded either relative
 * to the front of the robot, or relative to a certain absolute position. These
 * two modes are called "robot relative" and "field relative" respectively. The
 * drivetrain can also be commanded to rotate at a given speed independent of
 * its translation.
 */
class Drivetrain : public frc2::SubsystemBase {
private:
  enum module_id {
    kFrontLeft = 0,
    kFrontRight,
    kRearLeft,
    kRearRight,
    kNumModules
  };

public:
  using module_states_t = wpi::array<frc::SwerveModuleState, kNumModules>;

  using linear_cmd_supplier_t = std::function<units::meters_per_second_t()>;

  using rotation_cmd_supplier_t =
      std::function<units::revolutions_per_minute_t()>;

  using pose_supplier_t = std::function<frc::Pose2d()>;

  using chassis_speed_supplier_t = std::function<frc::ChassisSpeeds()>;

public:
  // The ctor of the Drivetrain subsystem.
  Drivetrain();

  // Need to define destructor to make simulation code compile
  ~Drivetrain();

  // Updates the odometer and SmartDashboard.
  void Periodic() override;

  // Executes the simulation
  void SimulationPeriodic() override;

  // Executes given command velocity (x, y, omega)
  // Motion is relative to the robot's frame
  // This is useful when a driver is looking through a camera
  void RobotRelativeDrive(const frc::ChassisSpeeds &cmd_vel);

  // Executes given command velocity (x, y, omega)
  // X and Y velocities are relative to the field coordinates.
  void Drive(const frc::ChassisSpeeds &cmd_vel);

  // Sets the state of each swerve module.
  void SetModuleStates(const module_states_t &desiredStates);

  // Returns the heading of the robot.
  frc::Rotation2d GetHeading();

  frc::Rotation2d GetGyroHeading();

  // Zeroes the robot heading.
  void ZeroHeading();

  void ZeroAbsEncoders();

  void SetAbsEncoderOffset();

  void SyncEncoders();

  void CoastMode(bool coast);

  void DriveToPose(const frc::Pose2d &desiredPose,
                   frc::ChassisSpeeds feedForward = {0_mps, 0_mps, 0_rpm},
                   const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg});

  // Returns the rotational velocity of the robot in degrees per second.
  units::degrees_per_second_t GetTurnRate();

  // Returns the uncorrected odometry transform for streaming to ROS
  frc::Pose2d GetOdomPose();

  // Returns the timestamp associated with the current odometry pose
  units::second_t GetOdomTimestamp();

  // Returns the robot heading and translation as a Pose2d.
  frc::Pose2d GetPose();

  frc::Pose2d GetSimulatedGroundTruth();

  // Add Vision Pose to SwerveDrivePoseEstimator.
  void AddVisionPoseEstimate(frc::Pose2d pose, units::second_t timestamp,
                             wpi::array<double, 3U> visionMeasurementStdDevs);

  // Returns Current Chassis Speed
  frc::ChassisSpeeds GetChassisSpeed();

  units::meters_per_second_t GetSpeed();

  bool AtPose(const frc::Pose2d &desiredPose,
              const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 2_deg});

  bool IsStopped();

  void ResetOdometry(const frc::Pose2d &pose);

  void SetMapToOdom(const frc::Transform2d &transform);

  // Display useful information on Shuffleboard.
  void InitializeDashboard();
  void UpdateDashboard();
  frc::Field2d &GetField() { return m_field; }

  // Drive the robot with swerve controls.
  frc2::CommandPtr RobotRelativeSwerveCommand(chassis_speed_supplier_t cmd_vel);

  // Drive the robot with field-relative swerve controls.
  frc2::CommandPtr BasicSwerveCommand(chassis_speed_supplier_t cmd_vel);

  frc2::CommandPtr Stop() {
    return BasicSwerveCommand([] { return frc::ChassisSpeeds{}; });
  }

  frc2::CommandPtr DynamicOdomReset();

  // Drives the robot to 'desiredPose()' with feedforward 'endVele);o'
  // until its within 'tolerance' of 'desiredPose'
  frc2::CommandPtr
  DriveToPoseCommand(pose_supplier_t desiredPoseSupplier,
                     frc::ChassisSpeeds feedForward = {0_mps, 0_mps, 0_rpm},
                     const frc::Pose2d &tolerance = {0.02_m, 0.02_m, 1.5_deg}) {
    return this
        ->RunEnd(
            [=, this] {
              DriveToPose(desiredPoseSupplier(), feedForward, tolerance);
            },
            [this] {
              m_field.GetObject("Desired Pose")->SetPose({80_m, 80_m, 0_deg});
            })
        .BeforeStarting([this] {
          // Without this, it will rotate towards a previous setpoint
          m_holonomicController.GetThetaController().Reset(
              GetHeading().Radians());
        })
        .Until([=, this] {
          return AtPose(desiredPoseSupplier(), tolerance) && IsStopped();
        });
  };

  frc2::CommandPtr
  DriveToPoseCommand(const frc::Pose2d &desiredPose,
                     frc::ChassisSpeeds feedForward = {0_mps, 0_mps, 0_rpm},
                     const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg}) {
    return DriveToPoseCommand([desiredPose] { return desiredPose; },
                              feedForward, tolerance);
  }

  // Drives the robot toward 'desiredPose()'
  // Warning: This command will not terminate unless interrupted,
  // or until the specified timeout.
  // Should be bound to triggers with WhileTrue
  frc2::CommandPtr
  DriveToPoseIndefinitelyCommand(pose_supplier_t desiredPoseSupplier,
                                 units::second_t timeout = 3.0_s) {
    return DriveToPoseCommand(std::move(desiredPoseSupplier),
                              {0_mps, 0_mps, 0_rpm}, {})
        .WithTimeout(timeout);
  }

  frc2::CommandPtr
  DriveToPoseIndefinitelyCommand(const frc::Pose2d &desiredPose,
                                 units::second_t timeout = 3.0_s) {
    return DriveToPoseIndefinitelyCommand([desiredPose] { return desiredPose; },
                                          timeout);
  }

  frc2::CommandPtr
  FollowPathCommand(pose_supplier_t desiredPoseSupplier,
                    const std::vector<frc::Translation2d> &waypoints,
                    units::meters_per_second_t endVelo = 0.0_mps,
                    const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg});

  frc2::CommandPtr
  FollowPathCommand(const frc::Pose2d &desiredPose,
                    const std::vector<frc::Translation2d> &waypoints,
                    units::meters_per_second_t endVelo = 0.0_mps,
                    const frc::Pose2d &tolerance = {0.06_m, 0.06_m, 3_deg}) {
    return FollowPathCommand([desiredPose] { return desiredPose; }, waypoints,
                             endVelo, tolerance);
  }

  frc2::CommandPtr
  FollowPathCommand(choreo::Trajectory<choreo::SwerveSample> trajectory,
                    bool isRed);

  /* Constructs a swerve control command from 3 independent controls
   * Each 'cmd' can be one of the following:
   * - A velocity (ie meters_per_second_t or radians_per_second_t)
   * - A function that returns a velocity
   * - A position setpoint (ie a meter_t or radian_t)
   * - A function that returns a position setpoint
   *
   * When given a setpoint instead of a velocity, the PID controller
   * used in DriveToPose is used to control that axis.
   *
   * This can be used to let a user control 1 axis while another axis
   * is tied to a position or sightline, such as auto-aiming.
   *
   * Example usage:
   * // Spin into a position
   * CustomSwerveCommand(10_m, 10_m, 30_rpm)
   *
   * // Lock heading
   * CustomSwerveCommand(
   *  [joy] {return joy.forward();},
   *  [joy] {return joy.strafe();},
   *  [] {return 100_deg;})
   */
  template <LinearCmd XCmd, LinearCmd YCmd, RotationCmd ThetaCmd>
  frc2::CommandPtr CustomSwerveCommand(XCmd &&x_cmd, YCmd &&y_cmd,
                                       ThetaCmd &&theta_cmd) {
    return BasicSwerveCommand(
        [forward = x_speed(std::forward<XCmd>(x_cmd)),
         strafe = y_speed(std::forward<YCmd>(y_cmd)),
         rot = theta_speed(std::forward<ThetaCmd>(theta_cmd))] {
          return frc::ChassisSpeeds{forward(), strafe(), rot()};
        });
  }

  /* @see CustomSwerveCommand
   * This function works the same as CustomSwerveCommand, but instead uses
   * the robot relative command.
   */
  template <LinearCmd XCmd, LinearCmd YCmd, RotationCmd ThetaCmd>
  frc2::CommandPtr CustomRobotRelativeSwerveCommand(XCmd &&x_cmd, YCmd &&y_cmd,
                                                    ThetaCmd &&theta_cmd) {
    return RobotRelativeSwerveCommand(
        [forward = x_speed(std::forward<XCmd>(x_cmd)),
         strafe = y_speed(std::forward<YCmd>(y_cmd)),
         rot = theta_speed(std::forward<ThetaCmd>(theta_cmd))] {
          return frc::ChassisSpeeds{forward(), strafe(), rot()};
        });
  }

  /* @see CustomRobotRelativeSwerveCommand
   * This overload allows passing in a bundle of all 3 cmds in
   * a single package, in x, y, omega order.
   *
   * Uses Robot Relative Swerve Command to Control Robot.
   * Example usage:
   * // static speed control
   * CustomSwerveCommand(frc::ChassisSpeeds{...})
   *
   * // dynamic speed control
   * CustomSwerveCommand([] {return frc::ChassisSpeeds{...};})
   *
   * // drive to dynamic pose
   * CustomSwerveCommand([] {return std::tuple{x, y, theta};})
   */
  template <typename TwistCmd>
  frc2::CommandPtr CustomRobotRelativeSwerveCommand(TwistCmd &&twist_cmd) {
    return RobotRelativeSwerveCommand(
        [this, twist_cmd = robot_twist(std::forward<TwistCmd>(twist_cmd))] {
          auto &&[x_cmd, y_cmd, theta_cmd] = twist_cmd();
          return frc::ChassisSpeeds{
              x_speed(std::forward<decltype(x_cmd)>(x_cmd))(),
              y_speed(std::forward<decltype(y_cmd)>(y_cmd))(),
              theta_speed(std::forward<decltype(theta_cmd)>(theta_cmd))()};
        });
  }

  /* @see CustomSwerveCommand
   * This overload allows passing in a bundle of all 3 cmds in
   * a single package, in x, y, omega order.
   * Example usage:
   * // static speed control
   * CustomSwerveCommand(frc::ChassisSpeeds{...})
   *
   * // dynamic speed control
   * CustomSwerveCommand([] {return frc::ChassisSpeeds{...};})
   *
   * // drive to dynamic pose
   * CustomSwerveCommand([] {return std::tuple{x, y, theta};})
   */
  template <typename TwistCmd>
  frc2::CommandPtr CustomSwerveCommand(TwistCmd &&twist_cmd) {
    return BasicSwerveCommand(
        [this, twist_cmd = robot_twist(std::forward<TwistCmd>(twist_cmd))] {
          auto &&[x_cmd, y_cmd, theta_cmd] = twist_cmd();
          return frc::ChassisSpeeds{
              x_speed(std::forward<decltype(x_cmd)>(x_cmd))(),
              y_speed(std::forward<decltype(y_cmd)>(y_cmd))(),
              theta_speed(std::forward<decltype(theta_cmd)>(theta_cmd))()};
        });
  }

  template <LinearCmd XCmd, LinearCmd YCmd>
  frc2::CommandPtr ZTargetCommand(XCmd &&x_cmd, YCmd &&y_cmd,
                                  pose_supplier_t target) {
    return CustomSwerveCommand(
        std::forward<XCmd>(x_cmd), std::forward<YCmd>(y_cmd), [this, target] {
          return (target().Translation() - GetPose().Translation())
              .Angle()
              .Radians();
        });
  }

  template <LinearCmd XCmd, LinearCmd YCmd>
  frc2::CommandPtr ZTargetCommand(XCmd &&x_cmd, YCmd &&y_cmd,
                                  const frc::Pose2d &target) {
    return ZTargetCommand(std::forward<XCmd>(x_cmd), std::forward<YCmd>(y_cmd),
                          [target] { return target; });
  }

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  frc2::CommandPtr ZeroAbsEncodersCommand();

  frc2::CommandPtr SetAbsEncoderOffsetCommand();

  frc2::CommandPtr CoastModeCommand(bool coast);

  frc2::CommandPtr ConfigAbsEncoderCommand();

private:
  frc::SwerveDriveKinematics<4> kDriveKinematics;
  std::array<SwerveModule, kNumModules> m_modules;

  studica::AHRS m_gyro;

  frc::PowerDistribution m_pdh;

  OdometryThread m_odom_thread;
  frc::Transform2d m_initial_transform; //< initial pose if known
  frc::Transform2d m_map_to_odom;       //< pose correction from sensors

  // Pose Estimator for estimating the robot's position on the field.
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;

  // Stores controllers for each motion axis
  frc::HolonomicDriveController m_holonomicController;

  frc::TrajectoryConfig m_trajConfig;

  // For placement on Dashboard
  frc2::CommandPtr zeroEncodersCommand{ZeroAbsEncodersCommand()};
  frc2::CommandPtr resetOdomCommand{DynamicOdomReset()};

  units::meters_per_second_squared_t prevAccel{0_mps_sq};

private:
  friend class DrivetrainSimulation;
  std::unique_ptr<DrivetrainSimulation> m_sim_state;

private:
  // magic to make doing stuff for every module easier
  auto each_module(auto &&fn) {
    return std::apply(
        [&fn](auto &&...ms) {
          return wpi::array{std::forward<decltype(fn)>(fn)(
              std::forward<decltype(ms)>(ms))...};
        },
        m_modules);
  }

  auto each_position() {
    return each_module([](SwerveModule &m) { return m.GetPosition(); });
  }

  auto each_state() {
    return each_module([](SwerveModule &m) { return m.GetState(); });
  }

private:
  // By defining these for several input types, AssistedDriveCommand
  // can take in any permutation of static setpoints, dynamic setpoints,
  // or dynamic control inputs and generate a command that does it

  // Runs the PID controller for a dynamic position
  auto x_speed(LinearPositionSupplier auto &&position) {
    return [this, position = std::forward<decltype(position)>(position)] {
      return units::meters_per_second_t{
          m_holonomicController.GetXController().Calculate(
              units::meter_t{GetPose().X()}.value(), position().value())};
    };
  }
  auto y_speed(LinearPositionSupplier auto &&position) {
    return [this, position = std::forward<decltype(position)>(position)] {
      return units::meters_per_second_t{
          m_holonomicController.GetYController().Calculate(
              units::meter_t{GetPose().Y()}.value(), position().value())};
    };
  }
  auto theta_speed(RotationSupplier auto &&heading) {
    return [this, heading = std::forward<decltype(heading)>(heading)] {
      return units::radians_per_second_t{
          m_holonomicController.GetThetaController().Calculate(
              GetPose().Rotation().Radians(), heading())};
    };
  }

  // Runs the PID controller for a static position
  auto x_speed(Distance auto position) {
    return x_speed([position] { return position; });
  }
  auto y_speed(Distance auto position) {
    return y_speed([position] { return position; });
  }
  auto theta_speed(Rotation auto heading) {
    return theta_speed([heading] { return heading; });
  }

  // Passes through the speed
  auto x_speed(LinearVelocitySupplier auto &&velocity) {
    return std::forward<decltype(velocity)>(velocity);
  }
  auto y_speed(LinearVelocitySupplier auto &&velocity) {
    return std::forward<decltype(velocity)>(velocity);
  }
  auto theta_speed(AngularVelocitySupplier auto &&velocity) {
    return std::forward<decltype(velocity)>(velocity);
  }

  auto x_speed(LinearVelocity auto velocity) {
    return [velocity] { return velocity; };
  }
  auto y_speed(LinearVelocity auto velocity) {
    return [velocity] { return velocity; };
  }
  auto theta_speed(AngularVelocity auto velocity) {
    return [velocity] { return velocity; };
  }

  // Combines x and y in one
  auto position_speed(std::invocable auto &&position) {
    return std::forward<decltype(position)>(position);
  }

  auto position_speed(auto &&position) {
    return [position = std::forward<decltype(position)>(position)] {
      return position();
    };
  }

  // Combines all 3 commands in one
  auto robot_twist(std::invocable auto &&twist) {
    return std::forward<decltype(twist)>(twist);
  }

  auto robot_twist(auto &&twist) {
    return [twist = std::forward<decltype(twist)>(twist)] { return twist; };
  }
};

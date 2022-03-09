package frc.robot;

import frc.robot.Configrun;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Static method containing all constant values for the robot in one location
 */
public final class Constants {

  /**
   * Static method containing all Drivetrain constants
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = (Configrun.get(3, "frontLeftDriveMotorPort")); // CANID of the
                                                                                                      // Translation
                                                                                                      // SparkMAX
    public static final int kFrontRightDriveMotorPort = (Configrun.get(1, "frontRightDriveMotorPort")); // CANID of the
                                                                                                        // Translation
                                                                                                        // SparkMAX
    public static final int kBackLeftDriveMotorPort = (Configrun.get(5, "backLeftDriveMotorPort")); // CANID of the
                                                                                                    // Translation
                                                                                                    // SparkMAX
    public static final int kBackRightDriveMotorPort = (Configrun.get(7, "backRightDriveMotorPort")); // CANID of the
                                                                                                      // Translation
                                                                                                      // SparkMAX

    public static final int kFrontLeftTurningMotorPort = (Configrun.get(4, "frontLeftTurningMotorPort")); // CANID of
                                                                                                          // the
                                                                                                          // Rotation
                                                                                                          // SparkMAX
    public static final int kFrontRightTurningMotorPort = (Configrun.get(2, "frontRightTurningMotorPort")); // CANID of
                                                                                                            // the
                                                                                                            // Rotation
                                                                                                            // SparkMAX
    public static final int kBackLeftTurningMotorPort = (Configrun.get(6, "backLeftTurningMotorPort")); // CANID of the
                                                                                                        // Rotation
                                                                                                        // SparkMAX
    public static final int kBackRightTurningMotorPort = (Configrun.get(8, "backRightTurningMotorPort")); // CANID of
                                                                                                          // the
                                                                                                          // Rotation
                                                                                                          // SparkMAX

    public static final int kFrontLeftTurningEncoderPort = (Configrun.get(1, "frontLeftTurningEncoderPort")); // Analog
                                                                                                              // Port of
                                                                                                              // the
                                                                                                              // Module
                                                                                                              // Absolute
                                                                                                              // Encoder
    public static final int kFrontRightTurningEncoderPort = (Configrun.get(0, "frontRightTurningEncoderPort")); // Analog
                                                                                                                // Port
                                                                                                                // of
                                                                                                                // the
                                                                                                                // Module
                                                                                                                // Absolute
                                                                                                                // Encoder
    public static final int kBackLeftTurningEncoderPort = (Configrun.get(3, "backLeftTurningEncoderPort")); // Analog
                                                                                                            // Port of
                                                                                                            // the
                                                                                                            // Module
                                                                                                            // Absolute
                                                                                                            // Encoder
    public static final int kBackRightTurningEncoderPort = (Configrun.get(2, "backRightTurningEncoderPort")); // Analog
                                                                                                              // Port of
                                                                                                              // the
                                                                                                              // Module
                                                                                                              // Absolute
                                                                                                              // Encoder

    public static final double kFrontLeftOffset = (Configrun.get(0.0, "frontLeftOffset")); // Encoder Offset in Radians
    public static final double kFrontRightOffset = (Configrun.get(0.7853, "frontRightOffset")); // Encoder Offset in
                                                                                                // Radians
    public static final double kBackLeftOffset = (Configrun.get(-0.0884, "backLeftOffset")); // Encoder Offset in
                                                                                             // Radians
    public static final double kBackRightOffset = (Configrun.get(2.79, "backRightOffset")); // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.0150, 0.2850, 0.25, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150, 0.2850, 0.25, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150, 0.2850, 0.25, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150, 0.2850, 0.25, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.6446; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    public static final double[] kKeepAnglePID = { 0.800, 0, 0 }; // Defines the PID values for the keep angle PID

  }

  /**
   * Static method containing all Swerve Module constants
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 4.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                           // 100% power from 0%
    private static final double kTranslationGearRatio = 8.33333333; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.09845; // Wheel Diameter in meters, may need to be experimentally
                                                          // determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                  // the
                                                                                                                  // conversion
                                                                                                                  // factor
                                                                                                                  // of
                                                                                                                  // RPM
                                                                                                                  // of
                                                                                                                  // the
                                                                                                                  // translation
                                                                                                                  // motor
                                                                                                                  // to
                                                                                                                  // m/s
                                                                                                                  // at
                                                                                                                  // the
                                                                                                                  // floor

    // NOTE: You shoulds ALWAYS define a reasonable current limit when using
    // brushless motors
    // due to the extremely high stall current avaialble
    public static final int kDriveCurrentLimit = 40; // Limits Translation Motor Current to improve efficiency and
                                                     // reduce voltage drop (Lower numbers will reduce acceleration)
    public static final int kTurnCurrentLimit = 30; // Limits Rotation Motor Current, this is generally not an issue
                                                    // with NEOs/Falcons on 2910 swerve but may be if smaller brushed
                                                    // motors are used

    public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
  }

  /**
   * Static method containing all User I/O constants
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // When making use of multiple controllers for drivers each
                                                       // controller will be on a different port
    public static final int kOperatorControllerPort = 1; // When making use of multiple controllers for drivers each
                                                         // controller will be on a different port
  }

  /**
   * Static method containing all Global constants
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.0; // Sets a voltage compensation value ideally 12.0V
  }

  /**
   * Static method containing all shooterPID constants
   */
  public static final class ShooterConstants {
    
    //PID constants
    public static final double shooterKs = 12;
    public static final double shooterKv = 0.35;
    public static final double shooterKa = 0.06;
    public static final double velocityFeedForwardMultiplier = 2.9;
    public static final double shooterToleranceInRPMs = 100;

  }

  /**
   * Static method containing all Turret constants
   */
  public static final class TurretConstants {
    public static final int kTurretPort = 1; // CANID of the turret motor controller
    public static final int kTurretPotentiometerPort = 4; // Analog port of the turret analog potentiometer
    public static final double kTurretTolerance = 0.0139626; // allowable angle error in radians for the PIDSubsystem to
                                                             // report atSetpoint() to true
    public static final double[] kTurretPID = { 1.6, 0.0, 0 }; // Defines the PID values for rotation of the turret
    public static final double kStaticGain = 0.025; // Limits Integral term so as to not wind up values when making
                                                    // larger moves
    public static final double kTurretLow = 1.00; // Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kTurretHigh = 5.25; // Maximum angle in radians allowed (defines the turret deadzone)
  }

  /**
   * Static method containing all Autonomous constants
   */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 1.0;
    public static final double kMaxSpeed = 3.25; // Maximum Sustainable Drivetrain Speed under Normal Conditions &
                                                 // Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeed, kMaxAngularAccel); // Creates a trapezoidal motion for the auto rotational commands
  }
}
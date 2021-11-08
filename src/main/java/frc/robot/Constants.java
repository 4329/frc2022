package frc.robot;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 4;
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kBackRightTurningMotorPort = 8;

    public static final int kFrontLeftTurningEncoderPort = 1;
    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kBackLeftTurningEncoderPort = 2;
    public static final int kBackRightTurningEncoderPort = 3;

    public static final double kFrontLeftOffset = Math.PI-4.305;
    public static final double kFrontRightOffset = Math.PI-1.605;
    public static final double kBackLeftOffset = Math.PI-3.230;
    public static final double kBackRightOffset = Math.PI-4.761;


    public static final double kTrackWidth = 0.5588;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.6446;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 3.42;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;


    public static final double kMaxSpeedMetersPerSecond = 3.25;
    public static final double kMaxAngularSpeed = Math.PI;

    public static final double kInnerDeadband = 0.08;
    public static final double kOuterDeadband = 0.98;

    public static final double kSlewRate = 10.0;
  }
  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    private static final double kTranslationGearRatio = 8.33333333;
    private static final double kWheelDiameter = 0.09845;

    public static final double kVelocityFactor = (1.0/kTranslationGearRatio/60.0)*kWheelDiameter*Math.PI;

    public static final int kDriveCurrentLimit = 30;
    public static final int kTurnCurrentLimit = 20;

    public static final double[] kDrivePID = {0.00125,0,0};
    public static final double[] kTurnPID = {6.0,0,0};

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ShooterConstants {
    public static final int kFeederPort = 2;
    public static final double kFeederSpeed = -1.0;
    public static final double kFeederReverseSpeed = 0.5;
    public static final double kThroatSpeed = -0.75;

    public static final int kMotorPorts[] = {9,10};
    public static final double kShotHysteresis = 50.0;
    public static final double[] kShooterPID = {0.005,0,0};

    public static final int[] kLimitSwitchPorts = {8,9};
    
    public static final double kSVolts = 0.00;
    public static final double kVVoltSecondsPerRotation = 0.00197;
  }

  public static final class TurretConstants {
    public static final int kTurretPort = 1;
    public static final int kTurretPotentiometerPort = 4;
    public static final double kTurretTolerance = 0.8/180.0*Math.PI;
    public static final double[] kTurretPID = {2.0,2.0,0};
    public static final double kTurretILimit = 0.025;
    public static final double kTurretLow = 1.00;
    public static final double kTurretHigh = 5.25;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2.0;
    public static final double kPYController = 2.0;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}

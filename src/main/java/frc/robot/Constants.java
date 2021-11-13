package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 3;   //CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 1;  //CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 5;    //CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 7;   //CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 4;   //CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 2;  //CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 6;    //CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 8;   //CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 1;   //Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 0;  //Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 2;    //Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 3;   //Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -1.1634;  //Encoder Offset in Radians
    public static final double kFrontRightOffset = 1.5366;  //Encoder Offset in Radians
    public static final double kBackLeftOffset = -0.0884;   //Encoder Offset in Radians
    public static final double kBackRightOffset = -1.6194;  //Encoder Offset in Radians

    public static final double onFloorFFAdj = 1.021;        //Value to adjust tuned ff values for when robot weight is applied (if values are tuned "on blocks") 

    //Drive motor PID is best done on the roboRIO currently as the SparkMAX does not allow for static gain values on the PID controller, 
    //    these are necessary to have high accuracy when moving at extremely low RPMs
    public static final double[] kFrontLeftTuningVals =   {0.0072*0.83333*3,0.33993*0.83333*onFloorFFAdj,0.25,0};  //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kFrontRightTuningVals =  {0.0055*0.83333*3,0.33321*0.83333*onFloorFFAdj,0.25,1}; //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackLeftTuningVals =    {0.0085*0.83333*3,0.34095*0.83333*onFloorFFAdj,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackRightTuningVals =   {0.0065*0.83333*3,0.33241*0.83333*onFloorFFAdj,0.25,3};    //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    //NOTE: 2910 Swerve the wheels are not directly under the center of rotation (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588;  // Center distance between right and left wheels on robot
    public static final double kWheelBaseLength = 0.6446;   // Center distance between front and back wheels on robot
     
    //Because the swerve modules poisition does not change, define a global SwerveDriveKinematics for the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2), new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2), new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 3.25; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kInnerDeadband = 0.10; //This value should exceed the maximum value the analog stick may read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; //This value should be lower than the analog stick X or Y reading when aimed at a 45deg angle (Such that X and Y are are maximized simultaneously)
  }

  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 0.25;         //Time in seconds the translation motors are allowed to go from 0 to 100% output, helps reduce wheelspin
    private static final double kTranslationGearRatio = 8.33333333; //Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.09845;           //Wheel Diameter in meters, may need to be experimentally determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; //Calculates the conversion factor of RPM of the translation motor to m/s at the floor

    //NOTE: You shoulds ALWAYS define a reasonable current limit when using brushless motors 
    //      due to the extremely high stall current avaialble
    public static final int kDriveCurrentLimit = 40; //Limits Translation Motor Current to improve efficiency and reduce voltage drop (Lower numbers will reduce acceleration speed)
    public static final int kTurnCurrentLimit = 20;  //Limits Rotation Motor Current, this is generally not an issue with NEOs/Falcons on 2910 swerve but may be if smaller brushed motors are used

    public static final double[] kTurnPID = { 0.666, 0, 0 }; //Defines the PID values for rotation of the serve modules, should show some minor oscillation when no weight is loaded on the modules
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;    //When making use of multiple controllers for drivers each controller will be on a different port
    public static final int kOperatorControllerPort = 1;  //When making use of multiple controllers for drivers each controller will be on a different port
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.0;        //Sets a voltage compensation value, should not exceed typical voltage while operating
  }

  public static final class ShooterConstants {
    public static final int kFeederPort = 2;              //CANID of the Motor Controller for the Feeder Motor
    public static final double kFeederSpeed = -1.0;       //Motor % to command when feeding balls into the shooter
    public static final double kFeederReverseSpeed = 0.5; //Motor % to command when reverse the balls to "unjam"
    public static final double kThroatSpeed = -0.75;      //Motor % to command when preloading a ball into the throat

    public static final int kMotorPorts[] = { 9, 10 };            //CANID of the SparkMAXs for the shooter motors
    public static final double kShooterCurrentLimit = 60;         //Limits max current draw of each shooter motor (Lower numbers will increase recovery time and spin-up time)
    public static final double kShotRPMTolerance = 75.0;          //RPMs of error allowed before a ball can be fed into t he shooter
    public static final double[] kShooterPID = { 0.00045, 0, 0 }; //Defines PID values for the shooter 0.00045
    public static final double kShooterFF = 0.0001755;             //Defines shooter FeedForward Value, should be roughly equal to 1/MaxMotorRPM * MaxRPMVoltage / Compensation Voltage

    public static final int[] kLimitSwitchPorts = { 8, 9 };       //Limit Switch DIO ports for the feed throat functionality
  }

  public static final class TurretConstants {
    public static final int kTurretPort = 1;                    //CANID of the turret motor controller
    public static final int kTurretPotentiometerPort = 4;       //Analog port of the turret analog potentiometer
    public static final double kTurretTolerance = 0.0139626;    //allowable angle error in radians for the PIDSubsystem to report atSetpoint() to true
    public static final double[] kTurretPID = { 1.6, 2.0, 0 };  //Defines the PID values for rotation of the turret
    public static final double kTurretILimit = 0.025;           //Limits Integral term so as to not wind up values when making larger moves
    public static final double kTurretLow = 1.00;               //Minimum angle in radians allowed (defines the turret deadzone)
    public static final double kTurretHigh = 5.25;              //Maximum angle in radians allowed (defines the turret deadzone)
  }

  public static final class AutoConstants {
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;        //sets the maximum desired autonomous angular speed in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; //sets the maximum desired autonomous angular acceleration in radians per second

    public static final double kPXController = 2.0;     //Sets proportional gain in the X direction for the auto SwerveControllerCommand
    public static final double kPYController = 2.0;     //Sets proportional gain in the Y direction for the auto SwerveControllerCommand
    public static final double kPThetaController = 1;   //Sets proportional gain in the angular heading for the auto SwerveControllerCommand

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared); //Creates a trapezoidal motion for the auto rotational commands
  }
}
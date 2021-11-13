// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.*;

//Swerve Drivetrain class that extends the SubsystemBase which is used to allow command based programming
public class Drivetrain extends SubsystemBase {

  //Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
    DriveConstants.kKeepAnglePID[1],DriveConstants.kKeepAnglePID[2]);
  
  private double keepAngle = 0.0;       //Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0;    //Double to store the time since last rotation command
  private double lastRotTime = 0.0;     //Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0;  //Double to store the time since last translation command
  private double lastDriveTime = 0.0;   //Double to store the time of the last translation command

  private final Timer keepAngleTimer = new Timer(); //Creates timer used in the perform keep angle function

  //Creates a swerveModule object for the front left swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals);

  //Creates a swerveModule object for the front right swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals);

  //Creates a swerveModule object for the back left swerve module feeding in parameters from the constants file
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals);

  //Creates a swerveModule object for the back right swerve module feeding in parameters from the constants file
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals);
  
  //Creates an ahrs gyro (NavX) on the MXP port of the RoboRIO
  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  //Creates Odometry object to store the pose of the robot
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, ahrs.getRotation2d());

  //When drivetrain subsystem is constructed reset and start the keepAngleTimer, reset the gyro, and enable continuous input for the Keep Angle PID
  public Drivetrain() {
    keepAngleTimer.reset();
    keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    ahrs.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    rot = performKeepAngle(xSpeed,ySpeed,rot); //Calls the keep angle function to update the keep angle or rotate depending on driver input
    
    //creates an array of the desired swerve module states based on driver command and if the commands are field relative or not
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    //normalize wheel speeds so all individual states are scaled to achievable velocities
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond); 

    //Set the 4 swerve module desired states from the previous functions
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    //Update swerve drive odometry periodically so robot pose can be tracked
    updateOdometry();    

    //Calls get pose function which sends the Pose information to the SmartDashboard
    getPose();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */  
  public void updateOdometry() {
    m_odometry.update(ahrs.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * @return Rotation2d object containing Gyro angle
   */  
  public Rotation2d getGyro() {
    return ahrs.getRotation2d();
  }

    /**
   * Function created to retreieve and push the robot pose to the SmartDashboard for diagnostics
   * @return Pose2d object containing the X and Y position and the heading of the robot.
   */  
  public Pose2d getPose() {
    SmartDashboard.putNumber("Robot X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Robot Y", m_odometry.getPoseMeters().getY());
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose in which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  /**
   * Resets the drivetrain gyro, keepAngle, and odometry to fix "Gyro Drift".
   * Gyro will be reset to desired angle and then the odometry will be reset at that gyro angle as well
   * @param angle (in radians) that the robot will be reset to
   */  
  public void reset(double angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle);
    keepAngle = getGyro().getRadians();
    m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0.0)), ahrs.getRotation2d());
  }
  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the swerve drive kinematics.
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity 
   */  
  public ChassisSpeeds getChassisSpeed(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
    m_backRight.getState());
  }

  /**
   * Keep angle function is performed to combat drivetrain drift without the need of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the keepAngle value. This value is updated when the robot 
   * is rotated manually by the driver input
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot is the input drive rotation speed command
   */  
  private double performKeepAngle(double xSpeed, double ySpeed, double rot){
    double output = rot; //Output shouold be set to the input rot command unless the Keep Angle PID is called
    if(Math.abs(rot) >= DriveConstants.kMinRotationCommand){  //If the driver commands the robot to rotate set the last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if( Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand  
          || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommand){ //if driver commands robot to translate set the last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get()-lastRotTime;      //update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get()-lastDriveTime;  //update variable to the current time - the last drive time
    if(timeSinceRot < 0.5){                               //Update keepAngle up until 0.5s after rotate command stops to allow rotation move to finish
      keepAngle = getGyro().getRadians();
    }
    else if(Math.abs(rot) < DriveConstants.kMinRotationCommand && timeSinceDrive < 0.75){ //Run Keep angle pid until 0.75s after drive command stops to combat decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle);               //Set output command to the result of the Keep Angle PID 
    }
    return output;
  }
}
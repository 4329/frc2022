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

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {

  private final PIDController m_keepAnglePID = new PIDController(2.5, 0.0, 0.0);
  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private final Timer keepAngleTimer = new Timer();

  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals);

  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals);

  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals);

  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals);
      
  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, ahrs.getRotation2d());

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
    rot = performKeepAngle(xSpeed,ySpeed,rot);
    SmartDashboard.putNumber("Rotation Command", rot);
    ChassisSpeeds diagnose = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d());
    SmartDashboard.putNumber("Robot Desired Speed X", diagnose.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Desired Speed Y", diagnose.vyMetersPerSecond);
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(ahrs.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
  }

  public Rotation2d getGyro() {
    return ahrs.getRotation2d();
  }

  public Pose2d getPose() {
    updateOdometry();
    SmartDashboard.putNumber("Robot X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Robot Y", m_odometry.getPoseMeters().getY());
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  public void reset(double angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle);
    keepAngle = getGyro().getRadians();
    m_odometry.resetPosition(new Pose2d(0, 0, new Rotation2d(0.0)), ahrs.getRotation2d());
  }

  public ChassisSpeeds getChassisSpeed(){
    return m_kinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
    m_backRight.getState());
  }

  public double[] printTransEncoders(){
    double[] encoders = {m_frontLeft.getTranslationEncPosition(), m_frontRight.getTranslationEncPosition(), m_backLeft.getTranslationEncPosition(), m_backRight.getTranslationEncPosition()};
    SmartDashboard.putNumber("Front Left Encoder", encoders[0]);
    SmartDashboard.putNumber("Front Right Encoder", encoders[1]);
    SmartDashboard.putNumber("Back Left Encoder", encoders[2]);
    SmartDashboard.putNumber("Back Right Encoder", encoders[3]);

    return encoders;
  }

  private double performKeepAngle(double xSpeed, double ySpeed, double rot){
    double output = rot;
    if(Math.abs(rot)>=Math.pow(DriveConstants.kInnerDeadband,2)){
      lastRotTime = keepAngleTimer.get();
    }
    if( Math.abs(xSpeed)>=Math.pow(DriveConstants.kInnerDeadband,2) 
          || Math.abs(ySpeed)>=Math.pow(DriveConstants.kInnerDeadband,2)){
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get()-lastRotTime;
    timeSinceDrive = keepAngleTimer.get()-lastDriveTime;
    SmartDashboard.putNumber("Time Since Rot", timeSinceRot);
    SmartDashboard.putNumber("Time Since Drive", timeSinceDrive);
    if(timeSinceRot < 0.75){
      keepAngle = getGyro().getRadians();
    }
    else if(Math.abs(rot)<Math.pow(DriveConstants.kInnerDeadband,2) && timeSinceDrive < 0.75){
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle);
    }
    return output;
  }
}
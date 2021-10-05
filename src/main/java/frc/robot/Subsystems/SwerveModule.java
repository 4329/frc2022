// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
 
import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.Constants.*;

public class SwerveModule {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANEncoder m_driveEncoder;
  private final Potentiometer m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = 
    new PIDController(
      ModuleConstants.kDrivePID[0],
      ModuleConstants.kDrivePID[1],
      ModuleConstants.kDrivePID[2]);

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController =
      new PIDController(
        ModuleConstants.kTurnPID[0],
        ModuleConstants.kTurnPID[1],
        ModuleConstants.kTurnPID[2]);

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = 
    new SimpleMotorFeedforward(
      DriveConstants.ksVolts, 
      DriveConstants.kvVoltSecondsPerMeter, 
      DriveConstants.kaVoltSecondsSquaredPerMeter);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN output for the drive motor.
   * @param turningMotorChannel CAN output for the turning motor.
   * @param turningEncoderChannel Analog input for trning encoder
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double angularOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kVelocityFactor);
    m_driveMotor.setInverted(false);
    m_driveMotor.burnFlash();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnCurrentLimit);
    m_turningMotor.setInverted(false);
    m_turningMotor.burnFlash();

    m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel,2.0*Math.PI,angularOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoder()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = 
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getTurnEncoder(), state.angle.getRadians());

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    
    m_turningMotor.setVoltage(turnOutput);
  }

  public double getTurnEncoder(){
    return -1.0*m_turningEncoder.get();
  }

}
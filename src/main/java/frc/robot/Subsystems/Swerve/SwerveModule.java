// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import frc.robot.Constants.*;

public class SwerveModule {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANEncoder m_driveEncoder;
  //private final CANPIDController m_drivePID;
  private final Potentiometer m_turningEncoder;

  private final double moduleID;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0,0,0);
  private final SlewRateLimiter m_driveLimiter = new SlewRateLimiter(1.0/ModuleConstants.kTranslationRampRate);

  private double driveStaticGain = 0.0;
  private double driveFeedForward = 0.0;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kTurnPID[0],
      ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel     CAN output for the drive motor.
   * @param turningMotorChannel   CAN output for the turning motor.
   * @param turningEncoderChannel Analog input for trning encoder
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double angularOffset, double[] tuningVals) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);
    m_driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);
    m_driveMotor.setInverted(false);
    m_driveMotor.setClosedLoopRampRate(ModuleConstants.kTranslationRampRate);

/*     m_drivePID = m_driveMotor.getPIDController();
    m_drivePID.setFF(tuningVals[1]);
    m_drivePID.setP(tuningVals[2]);
    m_drivePID.setOutputRange(-1.0, 1.0); */
    
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kVelocityFactor);
    m_driveEncoder.setPosition(0.0);

    m_driveMotor.burnFlash();

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnCurrentLimit);
    m_turningMotor.setInverted(false);

    m_turningMotor.burnFlash();

    m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2.0 * Math.PI, angularOffset);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivePIDController.setP(tuningVals[2]);

    driveStaticGain = tuningVals[0];
    driveFeedForward = tuningVals[1];
    moduleID = tuningVals[3];
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
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double driveFF = driveStaticGain*Math.signum(state.speedMetersPerSecond)+driveFeedForward*state.speedMetersPerSecond;

    m_driveMotor.set(m_driveLimiter.calculate(driveOutput+driveFF));

    //m_drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    SmartDashboard.putNumber("Desired Speed" + moduleID, state.speedMetersPerSecond);
    SmartDashboard.putNumber("Actual Speed" + moduleID, m_driveEncoder.getVelocity());
    SmartDashboard.putNumber("Speed Error%" + moduleID, (m_driveEncoder.getVelocity()-state.speedMetersPerSecond)/state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurnEncoder(), state.angle.getRadians());

    //m_driveMotor.set(translationCommand.calculate(driveOutput/12.0 + driveFeedforward/12.0));

    m_turningMotor.set(turnOutput);
  }

  public double getTurnEncoder() {
    return -1.0 * m_turningEncoder.get();
  }

  public double getTranslationEncPosition(){
    return m_driveEncoder.getPosition();
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import frc.robot.Constants.*;

public class SwerveModule {

  //Our swerve modules use NEOs for both translation and rotation motors
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  //Create a CANEncoder object for the translation position and velocity
  private final CANEncoder m_driveEncoder;
  
  //Create a Potentiometer to store the output of the absolute encoder that tracks the angular position of the swerve module
  private final Potentiometer m_turningEncoder;

  //Creates a variable to store the moduleID for various tuning and debugging (Currently not being used)
  //This value should be passed into the class contructor as part of the "tuningVals" array
  private final double moduleID;

  // Creates a PIDController for the translation motor on the swerve module
  // The PID values should be passed into the class constructor via the "tuningVals" array where they will be set
  private final PIDController m_drivePIDController;

  // Creates a SlewRateLimiter for the translation motors to reduce shock load on the tranlsation gears and to reduce wheelspin
  // For 2910 swerve this can help reduce the wear down the teeth on the steel spur gears just before the bevel gears 
  private final SlewRateLimiter m_driveLimiter = new SlewRateLimiter(ModuleConstants.kTranslationRampRate);

  // Creates a SimpleMotorFeedForward for the translation motor on the swerve module
  // The static and feedforward gains should be passed into the class contructor via the "tuningCals" array
  private SimpleMotorFeedforward driveFeedForward;

  // Creates a PIDController for the control of the anglular position of the swerve module
  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kTurnPID[0],
      ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel     CAN ID for the drive motor.
   * @param turningMotorChannel   CAN ID for the turning motor.
   * @param turningEncoderChannel analog input for turning absolute encoder
   * @param angularOffset         module specific offset for the absolute encoder
   * @param tuningVals            double array containing tuning values for translation in the following format {StaticGain, FeedForward, Prop Gain, ModuleID}
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double angularOffset, double[] tuningVals) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);      //Define the drive motor as the SparkMAX with the input driveMotorChannel
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);        //Set current limit for the drive motor
    m_driveMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);    //Enable voltage compensation so feedforward and gains scale with bus voltage
    m_driveMotor.setInverted(false);                                              //Motor direction is not inverted
    m_driveEncoder = m_driveMotor.getEncoder();                                   //Obtain the driveEncoder from the drive SparkMAX
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kVelocityFactor);  //Set velocity conversion factor so that encoder and PID control is in terms of velocity in m/s
    m_driveMotor.burnFlash();                                                     //Write these parameters to the SparkMAX so we can be sure the values are correct

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);  //Define the drive motor as the SparkMAX with the input driveMotorChannel
    m_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnCurrentLimit);       //Set current limit for the drive motor
    m_turningMotor.enableVoltageCompensation(GlobalConstants.kVoltCompensation);  //Enable voltage compensation so gains scale with bus voltage
    m_turningMotor.setInverted(false);                                            //Motor direction is not inverted
    m_turningMotor.burnFlash();                                                   //Write these parameters to the SparkMAX so we can be sure the values are correct

    //Creates the analog potentiometer for the tracking of the swerve module position converted to the range of 0-2*PI in radians offset by the tuned module offset
    m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2.0 * Math.PI, angularOffset); 

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous so the PID will command the shortest path.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    //Creates the SimpleMotorFeedForward for the swerve module using the static and feedforward gains from the tuningVals array
    driveFeedForward = new SimpleMotorFeedforward(tuningVals[0], tuningVals[1]);

    //Creates the drive PIDController using the proportional gain from the tuningVals array
    m_drivePIDController = new PIDController(tuningVals[2], 0.0, 0.0);

    //Sets the moduleID to the value stored in the tuningVals array
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
    //Calculates the desired feedForward motor % from the current velocity and the static and feedforward gains
    final double driveFF = driveFeedForward.calculate(m_driveEncoder.getVelocity());
    //Set the drive motor to the sum of the feedforward calculation and PID calculation
    m_driveMotor.set(m_driveLimiter.calculate(driveOutput+driveFF));
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurnEncoder(), state.angle.getRadians());
    //Set the turning motor to this output value
    m_turningMotor.set(turnOutput);
  }

  /**
   * Obtains the negative of the turning absolute encoder value as this encoder reads opposite of the module rotation on 
   * 2910 MK2 swerve.
   *
   * @return the modified absolute encoder value.
   */
  public double getTurnEncoder() {
    return -1.0 * m_turningEncoder.get();
  }
}
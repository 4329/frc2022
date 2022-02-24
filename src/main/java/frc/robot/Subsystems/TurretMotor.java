/*package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretMotor extends SubsystemBase {
    private final TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
    private final AS5600EncoderPwm turretEncoder = new AS5600EncoderPwm(m_turretMotor.getSensorCollection());
    private TurretPID rotationController;

public void turretMove(double speed) {


    // If change in angle exceeds 90, drives wheel to opposite angle (+ or - 180
    // degrees) and reverses speed
    // This is done to keep the wheel from spinning all the way around when not
    // needed
    if (Math.abs(angle - turretEncoder.getPwmPosition()) > 90) {
      rotation = rotationController.controlRotationExceeds90(angle); // A PID loop is used to hold the angle
      speed = speed * -1;
    }
    // Otherwise drive wheel to angle with normal speed
    else {
      rotation = rotationController.controlRotationWithin90(angle); // A PID loop is used to hold the angle
    }

    module.translationMotor.set(speed);
    module.rotationMotor.set(rotation);
  }

}*/
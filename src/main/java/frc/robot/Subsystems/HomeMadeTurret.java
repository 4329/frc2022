//have turret tallon move on button press
//have turret move left and right based on encoder pulses
// use tx limelight value to stop the turret
package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.Swerve.SwerveModule;
import frc.robot.Utilities.TurretAngle;
import frc.robot.Subsystems.LimelightSubsystem;

public class HomeMadeTurret{
    private final TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
    private final AS5600EncoderPwm turretEncoder = new AS5600EncoderPwm(m_turretMotor.getSensorCollection());

    double turretAcceptableMin = turretEncoder.getPwmPosition();

    public void turretActivate(){
        if(LimelightSubsystem.checkTx() >= -5.0 || LimelightSubsystem.checkTx() <= 5.0 ){
            m_turretMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        }

        else
        {
            m_turretMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
            // -120
        
        }   // 3960
    }
}
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class TrackingTurretSubsystem extends SubsystemBase {
    private final CANSparkMax turretMotor = new CANSparkMax(Configrun.get(43, "turretID"), MotorType.kBrushless);
    private final SparkMaxPIDController pidController = turretMotor.getPIDController();
    private final RelativeEncoder turretEncoder = turretMotor.getEncoder();

    private boolean isTrackingTarget = false;
    private double desiredAngle = 0.0;

    public TrackingTurretSubsystem() {

        turretMotor.setIdleMode(IdleMode.kBrake);
        turretMotor.setInverted(true);
        turretMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (Math.PI / 2));
        turretMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (Math.PI + (Math.PI / 2)));
        turretMotor.setSmartCurrentLimit(20);
        turretMotor.enableVoltageCompensation(12.6);
        turretEncoder.setPosition(180);
        pidController.setP(0);
        pidController.setFF(0.00017);
        pidController.setSmartMotionMaxAccel(15000, 0);
        pidController.setSmartMotionMaxVelocity(5000, 0);
        pidController.setSmartMotionAllowedClosedLoopError(0.2, 0);
    }

    public void turretFollow(Pose2d currentPose) {

    }



}
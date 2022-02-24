/*ackage frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.Swerve.SwerveModule;
import frc.robot.Utilities.TurretAngle;

public class TurretPID extends SubsystemBase {

    private double output, error;
    PIDController pidController;

    public TurretPID() {
        pidController = new PIDController(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD);
        // pidController.setInputRange(0.0, 360.0);
        // pidController.setContinuous(true);
        // pidController.setOutputRange(-1.0, 1.0);
        pidController.setTolerance(TurretConstants.kTurretTolerance);
        pidController.enableContinuousInput(0, 360);
    }

    // Runs the PID loop for angles within delta 90
    public double controlRotationWithin90(double angle) {
        // The wheel is commanded directly to the requested angle
        pidController.setSetpoint(angle);
        output = pidController.calculate(module.getAngle());
        output = output / 90;
        // SmartDashboard.putNumber("Output" + module.getPotentiometerPort(), output);
        error = pidController.getPositionError();
        // SmartDashboard.putNumber("Error" + module.getPotentiometerPort(), error);
        return output;
    }

    // Runs the PID loop for angles that exceed delta 90
    public double controlRotationExceeds90(double angle) {
        // Rather than turning the wheel all the way around to the requested angle
        // we turn the wheel to the complementary angle and command it backwards
        angle = angle - 180;
        angle = TurretAngle.resolveAngle(angle);
        pidController.setSetpoint(angle);
        output = pidController.calculate(module.getAngle());
        output = output / 90;
        // SmartDashboard.putNumber("Output" + module.getPotentiometerPort(), output);
        error = pidController.getPositionError();
        // SmartDashboard.putNumber("Error" + module.getPotentiometerPort(), error);
        // SmartDashboard.putNumber("SwerveAngle" + module.getPotentiometerPort(),
        return output;
    }

}
*/
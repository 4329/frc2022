package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.*;

public class FaceTurret extends CommandBase {
    private final Turret m_turret;
    private final Drivetrain m_robotDrive;

    public FaceTurret(Turret turret, Drivetrain drive) {
        m_turret = turret;
        m_robotDrive = drive;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_turret.enable();
        m_turret.trackTarget(false);
    }

    @Override
    public void execute() {
        m_turret.setAngle(m_robotDrive.getGyro().getRadians());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.disable();
    }

}

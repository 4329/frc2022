package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;

import frc.robot.Constants.*;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class AutoDrive extends CommandBase {
    private Drivetrain m_robotDrive;
    private boolean fieldOrient = true;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
    
    /*
      - Not a complete command!
      A command that takes change in position as an input instead of velocity
    */
    public void dPos(double dT, double dX, double dY, double dZ) {
        m_robotDrive.drive(dX / dT, dY / dT, dZ / dT, fieldOrient);
    }

}
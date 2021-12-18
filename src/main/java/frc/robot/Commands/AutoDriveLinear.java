package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.*;

public class AutoDriveLinear extends CommandBase {

    private final TrapezoidProfile m_xProfile;
    private final TrapezoidProfile m_yProfile;
    private final TrapezoidProfile m_rotProfile;
    private final Drivetrain m_drivetrain;
    private final Timer m_timer = new Timer();
    private final double m_timeOut;
    private final Pose2d m_startPose;
    private final Pose2d m_finalPose;
    private final boolean m_fieldOrient;

    public AutoDriveLinear(Drivetrain drive, double xMeters, double yMeters, double finalVel, double finalHeading, double timeOut, boolean fieldOrient)
    {
        m_timeOut = timeOut;
        m_fieldOrient = fieldOrient;

        final double distance = Math.sqrt(Math.pow(xMeters,2)+Math.pow(yMeters,2));
        final double xScale = xMeters/(distance);
        final double yScale = yMeters/(distance);

        SmartDashboard.putNumber("xScale", xScale);
        SmartDashboard.putNumber("yScale", yScale);

        m_drivetrain = drive;

        m_startPose = m_drivetrain.getPose();
        m_finalPose = new Pose2d(m_startPose.getX()+xMeters, m_startPose.getY()+yMeters, new Rotation2d (finalHeading));

        m_xProfile = new TrapezoidProfile( 
            new Constraints(DriveConstants.kMaxSpeedMetersPerSecond*Math.abs(xScale), DriveConstants.kMaxAcceleration*Math.abs(xScale)), 
            new State(xMeters,finalVel*xScale),
            new State(0.0,m_drivetrain.getChassisSpeed().vxMetersPerSecond));
        m_yProfile = new TrapezoidProfile(
            new Constraints(DriveConstants.kMaxSpeedMetersPerSecond*Math.abs(yScale), DriveConstants.kMaxAcceleration*Math.abs(yScale)), 
            new State(yMeters,finalVel*yScale),
            new State(0.0,m_drivetrain.getChassisSpeed().vyMetersPerSecond));
        m_rotProfile = new TrapezoidProfile(
            new Constraints(DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularAccel),
            new State(finalHeading,0),
            new State(m_startPose.getRotation().getRadians(),m_drivetrain.getChassisSpeed().omegaRadiansPerSecond));

    }
    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        final double time = m_timer.get();
        final double xCommand = m_xProfile.calculate(time).velocity;
        final double yCommand = m_yProfile.calculate(time).velocity;
        final double rotCommand = m_rotProfile.calculate(time).velocity;


        SmartDashboard.putNumber("xAutoCommand", xCommand);
        SmartDashboard.putNumber("yAutoCommand", yCommand);
        SmartDashboard.putNumber("rotAutoCommand", rotCommand);
        SmartDashboard.putNumber("AutoTimer", time);

        m_drivetrain.drive(xCommand,yCommand,rotCommand,m_fieldOrient);


        if(m_xProfile.isFinished(time) && m_yProfile.isFinished(time) && m_rotProfile.isFinished(time) || (time > m_timeOut))
        {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

}

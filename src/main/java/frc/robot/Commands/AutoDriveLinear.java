package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve.*;

public class AutoDriveLinear extends CommandBase {

    private final ProfiledPIDController m_xPID;
    private final ProfiledPIDController m_yPID;
    private final ProfiledPIDController m_rotPID;

    private final Drivetrain m_drivetrain;
    private final Timer m_timer = new Timer();
    private final double m_timeOut;
    private Pose2d m_startPose;
    private final boolean m_fieldOrient;
    private final State m_xPIDGoal;
    private final State m_yPIDGoal;
    private final State m_rotPIDGoal;

    public AutoDriveLinear(Drivetrain drive, double xMeters, double yMeters, double finalVel, double finalHeading,
            double timeOut, boolean fieldOrient) {

        m_timeOut = timeOut;
        m_fieldOrient = fieldOrient;

        final double distance = Math.sqrt(Math.pow(xMeters, 2) + Math.pow(yMeters, 2));
        final double xScale = xMeters / (distance);
        final double yScale = yMeters / (distance);

        SmartDashboard.putNumber("xScale", xScale);
        SmartDashboard.putNumber("yScale", yScale);

        m_drivetrain = drive;

        m_rotPID = new ProfiledPIDController(AutoConstants.kRotPropGain, 0.0, 0.0,
            new Constraints(AutoConstants.kMaxAngularSpeed, AutoConstants.kMaxAngularAccel));
        m_rotPID.setTolerance(AutoConstants.kRotTolerance);
        m_rotPID.enableContinuousInput(-Math.PI, Math.PI);
        m_rotPIDGoal = new State(finalHeading, 0.0); 
            
        m_xPID = new ProfiledPIDController(AutoConstants.kPosPropGain, 0.0, 0.0,
            new Constraints(AutoConstants.kMaxSpeedMetersPerSecond * Math.abs(xScale),
                AutoConstants.kMaxAcceleration * Math.abs(xScale)));
        m_xPID.setTolerance(AutoConstants.kPosTolerance);
        m_xPIDGoal = new State(xMeters, finalVel * xScale); 

        m_yPID = new ProfiledPIDController(AutoConstants.kPosPropGain, 0.0, 0.0,
        new Constraints(AutoConstants.kMaxSpeedMetersPerSecond * Math.abs(yScale),
            AutoConstants.kMaxAcceleration * Math.abs(yScale)));
        m_yPID.setTolerance(AutoConstants.kPosTolerance);
        m_yPIDGoal = new State(yMeters, finalVel * yScale); 

    }

    @Override
    public void initialize() {
        m_startPose = m_drivetrain.getPose();
        final ChassisSpeeds robotSpeed = m_drivetrain.getFieldRelativeSpeeds();
        m_xPID.reset(new State(m_startPose.getX(), robotSpeed.vxMetersPerSecond));
        m_yPID.reset(new State(m_startPose.getY(), robotSpeed.vyMetersPerSecond));
        m_rotPID.reset(new State(m_startPose.getRotation().getRadians(),robotSpeed.omegaRadiansPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        final double time = m_timer.get();
        final Pose2d currentPose = m_drivetrain.getPose();  
        final Pose2d expectedPose = new Pose2d(
            currentPose.getX()+m_xPID.getSetpoint().position,
            currentPose.getY()+m_yPID.getSetpoint().position, 
            new Rotation2d(currentPose.getRotation().getRadians() + m_rotPID.getSetpoint().position));

        final double xPIDoutput = m_xPID.calculate(currentPose.getX(), m_xPIDGoal);
        final double yPIDoutput = m_yPID.calculate(currentPose.getY(), m_yPIDGoal);
        final double rotPIDoutput = m_rotPID.calculate(currentPose.getRotation().getRadians(), m_rotPIDGoal);

        final double xDrive = xPIDoutput + m_xPID.getSetpoint().velocity;
        final double yDrive = yPIDoutput + m_yPID.getSetpoint().velocity;
        final double rotDrive = rotPIDoutput + m_rotPID.getSetpoint().velocity;


        SmartDashboard.putNumber("xAutoCommand", xDrive);
        SmartDashboard.putNumber("yAutoCommand", yDrive);
        SmartDashboard.putNumber("rotAutoCommand", rotDrive);
        SmartDashboard.putNumber("AutoTimer", time);

        m_drivetrain.drive(xDrive,yDrive,rotDrive,m_fieldOrient);

        final boolean atGoals = m_xPID.atGoal() && m_yPID.atGoal() && m_rotPID.atGoal();

        if(atGoals || (time > m_timeOut))
        {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

}

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Configrun;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.FieldRelativeAccel;
import frc.robot.Utilities.FieldRelativeSpeed;
import frc.robot.Utilities.MathUtils;

public class SwerveShotCommand extends CommandBase {

    Shooter shooter;
    HoodSubsystem hood;
    // TurretSubsystem turret;
    Drivetrain drivetrain;
    XboxController xboxController;
    ShooterFeedSubsytem shooterFeedSubsytem;
    StorageIntake storageIntake;
    TurretSubsystem turretSubsystem;
    Timer timer = new Timer();
    PIDController swervepid = new PIDController (5.0, 0, 0.2);
    NetworkTableEntry tGEntry = Shuffleboard.getTab("RobotData").add("tG", 1).getEntry();
    NetworkTableEntry tREntry = Shuffleboard.getTab("RobotData").add("tR", 1).getEntry();
    
    NetworkTableEntry pidGraph = Shuffleboard.getTab("RobotData").add("Pid Graph", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    public SwerveShotCommand(Shooter shooter, HoodSubsystem hood, /*TurretSubsystem turret,*/ Drivetrain drivetrain, XboxController xboxController, StorageIntake storageIntake, ShooterFeedSubsytem shooterFeedSubsytem, TurretSubsystem turretSubsystem) {

        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.hood = hood;
        // this.turret = turret;
        this.xboxController = xboxController;
        this.storageIntake = storageIntake;
        this.shooterFeedSubsytem = shooterFeedSubsytem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(shooter, drivetrain, hood, storageIntake, shooterFeedSubsytem, turretSubsystem);
        

        swervepid.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void initialize() {

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        if (Configrun.get(false, "extraShuffleBoardToggle")) {

            pidGraph.setDouble(swervepid.getPositionError());
        }

        double currentTime = timer.get();
        FieldRelativeSpeed speed = drivetrain.getRelativeSpeed();
        FieldRelativeAccel accel = drivetrain.getRelativeAccel();
        Translation2d goal = Constants.ShooterConstants.goalPos;
        Translation2d robotToGoal = goal.minus(drivetrain.getPose().getTranslation());
        double dist = MathUtils.MetersToInches(robotToGoal.getDistance(new Translation2d()));
        double shotTime = Constants.TuningConstants.kTimeTable.getOutput(dist);
        Translation2d movingGoal = new Translation2d();

        for (int i = 0; i < 5; i++) {

            double virtualGoalx = goal.getX()
                    - shotTime * (speed.vx + accel.ax * 0.1);

            double virtualGoalY = goal.getY()
                    - shotTime * (speed.vy + accel.ay * 0.1);

            Translation2d testGoal = new Translation2d(virtualGoalx, virtualGoalY);

            Translation2d robotToTestGoal = testGoal.minus(drivetrain.getPose().getTranslation());

            double newShotTime = Constants.TuningConstants.kTimeTable
                    .getOutput(MathUtils.MetersToInches(robotToTestGoal.getDistance(new Translation2d())));

            if (Math.abs(newShotTime - shotTime) <= 0.10) {

                i = 4;
            }

            if (i == 4) {

                movingGoal = testGoal;
            } else {

                shotTime = newShotTime;
            }
        }

        Translation2d robotToMovingGoal = movingGoal.minus(drivetrain.getPose().getTranslation());

        double newDistance = MathUtils.MetersToInches(robotToMovingGoal.getDistance(new Translation2d()));

        if (xboxController.getLeftTriggerAxis() > 0.5) {

            hood.setDesiredHoodPos(Constants.TuningConstants.m_hoodTable.getOutput(newDistance));

                if(hood.atSetpoint()) {
                    storageIntake.storageIntakeInSlow();
                    shooterFeedSubsytem.shooterFeedFire();
                }

                else {
                    storageIntake.storageIntakeStop();
                    shooterFeedSubsytem.shooterFeedStop();
                }
            }
         else {
                hood.setPosition(HoodPosition.OPEN);
                storageIntake.storageIntakeStop();
                shooterFeedSubsytem.shooterFeedStop();
            }

        shooter.shoot(Constants.TuningConstants.m_rpmTable.getOutput(newDistance));

        double targetAngle = Math.atan2(robotToMovingGoal.getY(), robotToMovingGoal.getX()) + Math.PI;
        targetAngle = MathUtils.toUnitCircAngle(targetAngle);
        double currentAngle = MathUtils.toUnitCircAngle(drivetrain.getGyro().getRadians());

        double pidOutput = swervepid.calculate(currentAngle, targetAngle);

        if (pidOutput > Constants.DriveConstants.kMaxAngularSpeed){

            pidOutput = Constants.DriveConstants.kMaxAngularSpeed;
        } else if (pidOutput < - Constants.DriveConstants.kMaxAngularSpeed){

            pidOutput = -Constants.DriveConstants.kMaxAngularSpeed;
        }

        double adjTranslation = ((Constants.DriveConstants.kMaxAngularSpeed - Math.abs(pidOutput)) / Constants.DriveConstants.kMaxAngularSpeed) * 0.5;

        drivetrain.drive(
            -inputTransform(xboxController.getLeftY())
            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
        -inputTransform(xboxController.getLeftX())
            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
        pidOutput,
        true);

        double angleError = MathUtils.toUnitCircAngle(targetAngle - currentAngle);
        double turretAngle = MathUtils.toUnitCircAngle(angleError + Math.PI) - Math.PI;
        turretSubsystem.setTurretAngle(/*turretAngle*/ 0);
        //TODO also make sure hood is not impeding the limlight
        if (currentTime > 0.250 && TurretSubsystem.targetVisible() && TurretSubsystem.getDistanceFromTarget() >= 85.0 && hood.getEncoderPos() < 4.0) {
            //double dL = TurretSubsystem.getDistanceFromTarget() * 0.0254;
            double dL = MathUtils.inchesToMeters(TurretSubsystem.getDistanceFromTarget());
            double tR = drivetrain.getGyro().getRadians();
            double tT = Math.PI + turretSubsystem.getTrueTurretPos();
            double tL = -1.0 * TurretSubsystem.getTx();

            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, Constants.ShooterConstants.goalPos);

            drivetrain.setPose(pose);
        }
    }

    @Override
    public void end(boolean interrupted) {

        shooter.holdFire();
        hood.setPosition(HoodPosition.OPEN);  
        storageIntake.storageIntakeStop();
        shooterFeedSubsytem.shooterFeedStop();
        turretSubsystem.turretStop();
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {

        double tG = tR + tT + Math.toRadians(tL);
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        tREntry.setDouble(tR);
        tGEntry.setDouble(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }

    private double inputTransform(double input) {

        return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    }

}
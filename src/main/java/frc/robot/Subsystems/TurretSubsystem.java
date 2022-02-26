package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import frc.robot.PIDConstants;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
//import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase{

    private TalonSRX turret;
    private RelativeEncoder turretEncoder;
    private double output, error;
    PIDController pidController;
    double staticFeedforward = Configrun.get(0.103, "turnStaticFeedforward");

    SmartDashboard table;
    SmartDashboard ledSmartDashboard;
    // default value for the limelight mode
    int defaultvalue = 1;
    PIDController limeLightPid;
    double h1In = Configrun.get(36, "h1In");
    // height of the limelight off of the ground "36 inches" this year
    double h2In = Configrun.get(104, "h2In");
    // height of the target "8ft 8inches" aka 104 inches this year
    double a1Degree = Configrun.get(0.0, "a1Degree");
    // angle of the front of the limelight in relation to level
    double limeLightDistance;
    int limeLightTolerance = Configrun.get(1, "limeLightTolerance");
    double limelightP = Configrun.get(1.2, "limelightP");
    double limelightI = Configrun.get(0, "limelightI");
    double limelightD = Configrun.get(0.125, "limelightD");
    double taTolerance;
    public double currentDistance = 120;

    private NetworkTableEntry checkTXDisplay;
    private NetworkTableEntry checkTYDisplay;
    private NetworkTableEntry checkTADisplay;
    private NetworkTableEntry getDistanceFromTargetDisplay;
    NetworkTableEntry targetStatus;


    public TurretSubsystem() {
        turret = new TalonSRX (Configrun.get(41, "turretID"));
        //turretEncoder = turret.getEncoder;
        limeLightPid = new PIDController(limelightP, limelightI, limelightD);
        taTolerance = Configrun.get(0.3, "taTolerance");
        limeLightPid.setTolerance(limeLightTolerance);
        targetStatus = Shuffleboard.getTab("RobotData").add("Target Acquired", false).getEntry();
        checkTXDisplay = Shuffleboard.getTab("limlight").add("Tx", 0).withPosition(3, 1).getEntry();
        checkTYDisplay = Shuffleboard.getTab("limlight").add("TY", 0).withPosition(3, 0).getEntry();
        checkTADisplay = Shuffleboard.getTab("limlight").add("TA", 0).withPosition(4, 0).getEntry();
        getDistanceFromTargetDisplay = Shuffleboard.getTab("limlight").add("Distance", 0).withPosition(5, 0).getEntry();
        pidController = new PIDController(PIDConstants.ROTATION_P, PIDConstants.ROTATION_I, PIDConstants.ROTATION_D);
        // pidController.setInputRange(0.0, 360.0);
        // pidController.setContinuous(true);
        // pidController.setOutputRange(-1.0, 1.0);
        pidController.setTolerance(PIDConstants.TOLERANCE);
        //pidController.enableContinuousInput(0, 360);
    }

    // public void setDistance() {
    //     currentDistance = getDistanceFromTarget();
    // }
    // Sets calculation distance for shooter

    public void putValuesToShuffleboard() {
        checkTXDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
        checkTYDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
        checkTADisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
        getDistanceFromTargetDisplay.setDouble(getDistanceFromTarget());
    }

    public double checkTx() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }
    // check Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

    public double checkTa() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }
    // checks the area visible of the Target (0% of image to 100% of image)

    public double checkTy() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }
    // checks Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)

    public double getDistanceFromTarget() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        return limeLightDistance;
    }

    public void limeLightOn() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(defaultvalue);
        // turns the limelight on using defaulvalue

    }

    public void limeLightStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(0);
    }

    public void putTargetAcquired() {
        boolean status;
        // true or false value
        if (checkTa() >= taTolerance) {
            status = true;
            // System.out.println("limelighttrue");
        } else {
            status = false;
            // System.out.println("limelightfalse");
        }
        targetStatus.setBoolean(status);
    }

    public void targeting() {
            double output = limeLightPid.calculate(checkTx(), 0);
            //30 is the limiting factor for the output
            output = output / 30;
            if (output < 0) {
                output = output - staticFeedforward;
            } 
            else {
                output = output + staticFeedforward;
            }
            System.out.println(output);
            // SmartDashboard.putNumber("LimelightOutput", output);

            // RobotContainer.swerveDrive.drive(0, 0, output);
            // setDistance();
    }

    // Runs the PID loop for angles within delta 90
    // public double controlRotationWithin90(double x, SwerveModule module) {
    //     // The wheel is commanded directly to the requested angle
    //     pidController.setSetpoint(x);
    //     output = pidController.calculate(.getAngle());
    //     output = output / 90;
    //     // SmartDashboard.putNumber("Output" + module.getPotentiometerPort(), output);
    //     error = pidController.getPositionError();
    //     // SmartDashboard.putNumber("Error" + module.getPotentiometerPort(), error);
    //     return output;
    // }

    public void turretRight(){
        turret.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "turretRightPower"));
    }

    public void turretLeft(){
        turret.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "turretLeftPower"));
    }

    public void turretStop(){
        turret.set(TalonSRXControlMode.PercentOutput, Configrun.get(0, "turretStop"));
    }

    // public void turretPeriodic(){
    //     double output = hoodPID.calculate(hoodposition, hoodSetpoint.getDouble(0));
    // output = output / MAX_RANGE;
    // setpointDifference = hoodSetpoint.getDouble(0) - hoodposition;
    // }
}

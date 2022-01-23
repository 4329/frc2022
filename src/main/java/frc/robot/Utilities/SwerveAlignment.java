package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class SwerveAlignment {
    private Drivetrain m_dDrivetrain;
    public static final String SWERVE_ALIGNMENT = "Swerve Alignment";

    //Front left
    private NetworkTableEntry frontLeftAngleDisplay;
    private NetworkTableEntry frontLeftRawAngleDisplay;
    private NetworkTableEntry frontLeftOffsetDisplay;
    private NetworkTableEntry frontLeftAlignmentDisplayDeg;
    private NetworkTableEntry frontLeftAlignmentDisplayRad;
    private double frontLeftInitialAngle;
    //Front right
    private NetworkTableEntry frontRightAngleDisplay;
    private NetworkTableEntry frontRightRawAngleDisplay;
    private NetworkTableEntry frontRightOffsetDisplay;
    private NetworkTableEntry frontRightAlignmentDisplayDeg;
    private NetworkTableEntry frontRightAlignmentDisplayRad;
    private double frontRightInitialAngle;
    //Back left
    private NetworkTableEntry backLeftAngleDisplay;
    private NetworkTableEntry backLeftRawAngleDisplay;
    private NetworkTableEntry backLeftOffsetDisplay;
    private NetworkTableEntry backLeftAlignmentDisplayDeg;
    private NetworkTableEntry backLeftAlignmentDisplayRad;
    private double backLeftInitialAngle;
    //Back right
    private NetworkTableEntry backRightAngleDisplay;
    private NetworkTableEntry backRightRawAngleDisplay;
    private NetworkTableEntry backRightOffsetDisplay;
    private NetworkTableEntry backRightAlignmentDisplayDeg;
    private NetworkTableEntry backRightAlignmentDisplayRad;
    private double backRightInitialAngle;

	public SwerveAlignment(Drivetrain drivetrain) {
        m_dDrivetrain = drivetrain;
	}
    //This section creates the widgets
    public void initSwerveAlignment(){
        //Documentation:
        Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Documentation","https://github.com/4329/frc2022/blob/main/swerveAlignmentDocs.md").withPosition(7,1).withSize(4,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        //Front left
        frontLeftInitialAngle = m_dDrivetrain.getFrontLeftAngle();
        frontLeftAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Front Left Angle",m_dDrivetrain.getFrontLeftAngle()).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FL Raw Angle",m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset).withPosition(1,0).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Front Left Offset",DriveConstants.kFrontLeftOffset).withPosition(2,0).withSize(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FL Test Offset (d)",Math.toDegrees(frontLeftInitialAngle - m_dDrivetrain.getFrontLeftAngle())).withPosition(0,1).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FL Test Offset (r)",frontLeftInitialAngle - m_dDrivetrain.getFrontLeftAngle()).withPosition(1,1).withWidget(BuiltInWidgets.kTextView).getEntry();
        //Front right
        frontRightInitialAngle = m_dDrivetrain.getFrontRightAngle();
        frontRightAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Front Right Angle",m_dDrivetrain.getFrontLeftAngle()).withPosition(3,0).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FR Raw Angle",m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset).withPosition(4,0).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Front Right Offset",DriveConstants.kFrontRightOffset).withPosition(5,0).withSize(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FR Test Offset (d)",Math.toDegrees(frontRightInitialAngle - m_dDrivetrain.getFrontRightAngle())).withPosition(3,1).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("FR Test Offset (r)",frontRightInitialAngle - m_dDrivetrain.getFrontRightAngle()).withPosition(4,1).withWidget(BuiltInWidgets.kTextView).getEntry();
        //Back left
        backLeftInitialAngle = m_dDrivetrain.getBackLeftAngle();
        backLeftAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Back Left Angle",m_dDrivetrain.getBackLeftAngle()).withPosition(0,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BL Raw Angle",m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset).withPosition(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Back Left Offset",DriveConstants.kBackLeftOffset).withPosition(2,2).withSize(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BL Test Offset (d)",Math.toDegrees(backLeftInitialAngle - m_dDrivetrain.getBackLeftAngle())).withPosition(0,3).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BL Test Offset (r)",backLeftInitialAngle - m_dDrivetrain.getBackLeftAngle()).withPosition(1,3).withWidget(BuiltInWidgets.kTextView).getEntry();
        //Back right
        backRightInitialAngle = m_dDrivetrain.getBackRightAngle();
        backRightAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Back Right Angle",m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset).withPosition(3,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BR Raw Angle",m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset).withPosition(4,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Back Right Offset",DriveConstants.kBackRightOffset).withPosition(5,2).withSize(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BR Test Offset (d)",Math.toDegrees(backRightInitialAngle - m_dDrivetrain.getBackRightAngle())).withPosition(3,3).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT).add("BR Test Offset (r)",backRightInitialAngle - m_dDrivetrain.getBackRightAngle()).withPosition(4,3).withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    //This section constantly updates the values displayed in the widgets
    public void updateSwerveAlignment()
    {
        //Front left
        frontLeftAngleDisplay.setDouble(m_dDrivetrain.getFrontLeftAngle());
        frontLeftRawAngleDisplay.setDouble(m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset);
        frontLeftOffsetDisplay.setDouble(DriveConstants.kFrontLeftOffset);
        frontLeftAlignmentDisplayDeg.setDouble(Math.toDegrees(frontLeftInitialAngle - m_dDrivetrain.getFrontLeftAngle()));
        frontLeftAlignmentDisplayRad.setDouble(frontLeftInitialAngle - m_dDrivetrain.getFrontLeftAngle());
        //Front right
        frontRightAngleDisplay.setDouble(m_dDrivetrain.getFrontRightAngle());
        frontRightRawAngleDisplay.setDouble(m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset);
        frontRightOffsetDisplay.setDouble(DriveConstants.kFrontRightOffset);
        frontRightAlignmentDisplayDeg.setDouble(Math.toDegrees(frontRightInitialAngle - m_dDrivetrain.getFrontRightAngle()));
        frontRightAlignmentDisplayRad.setDouble(frontRightInitialAngle - m_dDrivetrain.getFrontRightAngle());
        //Back left
        backLeftAngleDisplay.setDouble(m_dDrivetrain.getBackLeftAngle());
        backLeftRawAngleDisplay.setDouble(m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset);
        backLeftOffsetDisplay.setDouble(DriveConstants.kBackLeftOffset);
        backLeftAlignmentDisplayDeg.setDouble(Math.toDegrees(backLeftInitialAngle - m_dDrivetrain.getBackLeftAngle()));
        backLeftAlignmentDisplayRad.setDouble(backLeftInitialAngle - m_dDrivetrain.getBackLeftAngle());
        //Back right
        backRightAngleDisplay.setDouble(m_dDrivetrain.getBackRightAngle());
        backRightRawAngleDisplay.setDouble(m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset);
        backRightOffsetDisplay.setDouble(DriveConstants.kBackRightOffset);
        backRightAlignmentDisplayDeg.setDouble(Math.toDegrees(backRightInitialAngle - m_dDrivetrain.getBackRightAngle()));
        backRightAlignmentDisplayRad.setDouble(backRightInitialAngle - m_dDrivetrain.getBackRightAngle());
    }
}

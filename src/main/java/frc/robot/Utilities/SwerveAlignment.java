package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class SwerveAlignment {
    private Drivetrain m_dDrivetrain;
    public static final String SWERVE_ALIGNMENT = "Swerve Alignment";

    // Front left
    private NetworkTableEntry frontLeftAngleDisplay;
    private NetworkTableEntry frontLeftRawAngleDisplay;
    private NetworkTableEntry frontLeftOffsetDisplay;
    private NetworkTableEntry frontLeftAlignmentDisplayDeg;
    private NetworkTableEntry frontLeftAlignmentDisplayRad;
    private double frontLeftInitialAngle;
    // Front right
    private NetworkTableEntry frontRightAngleDisplay;
    private NetworkTableEntry frontRightRawAngleDisplay;
    private NetworkTableEntry frontRightOffsetDisplay;
    private NetworkTableEntry frontRightAlignmentDisplayDeg;
    private NetworkTableEntry frontRightAlignmentDisplayRad;
    private double frontRightInitialAngle;
    // Back left
    private NetworkTableEntry backLeftAngleDisplay;
    private NetworkTableEntry backLeftRawAngleDisplay;
    private NetworkTableEntry backLeftOffsetDisplay;
    private NetworkTableEntry backLeftAlignmentDisplayDeg;
    private NetworkTableEntry backLeftAlignmentDisplayRad;
    private double backLeftInitialAngle;
    // Back right
    private NetworkTableEntry backRightAngleDisplay;
    private NetworkTableEntry backRightRawAngleDisplay;
    private NetworkTableEntry backRightOffsetDisplay;
    private NetworkTableEntry backRightAlignmentDisplayDeg;
    private NetworkTableEntry backRightAlignmentDisplayRad;
    private double backRightInitialAngle;

    /**
     * Creates a SwerveAlignment on Shuffleboard
     * 
     * @param drivetrain
     */
    public SwerveAlignment(Drivetrain drivetrain) {
        m_dDrivetrain = drivetrain;
    }

    // This section creates the widgets
    public void initSwerveAlignmentWidgets() {
        frontLeftInitialAngle = m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset;
        frontRightInitialAngle = m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset;
        backLeftInitialAngle = m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset;
        backRightInitialAngle = m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset;
        // Documentation:
        Shuffleboard.getTab(SWERVE_ALIGNMENT).add("Documentation",
                "stem2u.sharepoint.com/sites/frc-4329/_layouts/15/Doc.aspx?sourcedoc={ee32bd41-c1a3-423a-87d6-f53d8420ff36}&action=edit&wd=target%28Software.one%7C2a079b56-fdcc-4604-b12e-be6a0710f6e0%2FSwerve%20Alignment%20Documentation%7Cc3883e58-a23c-4335-9693-aaef763459b2%2F%29")
                .withPosition(7, 1).withSize(4, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        // Front left
        frontLeftAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Front Left Angle", m_dDrivetrain.getFrontLeftAngle()).withWidget(BuiltInWidgets.kTextView)
                .getEntry();
        frontLeftRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FL Raw Angle", m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset)
                .withPosition(1, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Front Left Offset", DriveConstants.kFrontLeftOffset).withPosition(2, 0).withSize(1, 2)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FL Test Offset (d)", Math.toDegrees(m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle))
                .withPosition(0, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontLeftAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FL Test Offset (r)", m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        // Front right
        frontRightAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Front Right Angle", m_dDrivetrain.getFrontLeftAngle()).withPosition(3, 0)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FR Raw Angle", m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset)
                .withPosition(4, 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Front Right Offset", DriveConstants.kFrontRightOffset).withPosition(5, 0).withSize(1, 2)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FR Test Offset (d)", Math.toDegrees(m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle))
                .withPosition(3, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
        frontRightAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("FR Test Offset (r)", m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle)
                .withPosition(4, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
        // Back left
        backLeftAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Back Left Angle", m_dDrivetrain.getBackLeftAngle()).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BL Raw Angle", m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset)
                .withPosition(1, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Back Left Offset", DriveConstants.kBackLeftOffset).withPosition(2, 2).withSize(1, 2)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BL Test Offset (d)", Math.toDegrees(m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle))
                .withPosition(0, 3).withWidget(BuiltInWidgets.kTextView).getEntry();
        backLeftAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BL Test Offset (r)", m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle).withPosition(1, 3)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        // Back right
        backRightAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Back Right Angle", m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset)
                .withPosition(3, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightRawAngleDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BR Raw Angle", m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset)
                .withPosition(4, 2).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightOffsetDisplay = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("Back Right Offset", DriveConstants.kBackRightOffset).withPosition(5, 2).withSize(1, 2)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightAlignmentDisplayDeg = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BR Test Offset (d)", Math.toDegrees(m_dDrivetrain.getBackRightAngle() - backRightInitialAngle))
                .withPosition(3, 3).withWidget(BuiltInWidgets.kTextView).getEntry();
        backRightAlignmentDisplayRad = Shuffleboard.getTab(SWERVE_ALIGNMENT)
                .add("BR Test Offset (r)", m_dDrivetrain.getBackRightAngle() - backRightInitialAngle).withPosition(4, 3)
                .withWidget(BuiltInWidgets.kTextView).getEntry();
    }

    // This section constantly updates the values displayed in the widgets
    public void updateSwerveAlignment() {
        // Front left
        frontLeftAngleDisplay.setDouble(m_dDrivetrain.getFrontLeftAngle());
        frontLeftRawAngleDisplay.setDouble(m_dDrivetrain.getFrontLeftAngle() - DriveConstants.kFrontLeftOffset);
        frontLeftOffsetDisplay.setDouble(DriveConstants.kFrontLeftOffset);
        frontLeftAlignmentDisplayDeg
                .setDouble(Math.toDegrees(m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle));
        frontLeftAlignmentDisplayRad.setDouble(m_dDrivetrain.getFrontLeftAngle() - frontLeftInitialAngle);
        // Front right
        frontRightAngleDisplay.setDouble(m_dDrivetrain.getFrontRightAngle());
        frontRightRawAngleDisplay.setDouble(m_dDrivetrain.getFrontRightAngle() - DriveConstants.kFrontRightOffset);
        frontRightOffsetDisplay.setDouble(DriveConstants.kFrontRightOffset);
        frontRightAlignmentDisplayDeg
                .setDouble(Math.toDegrees(m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle));
        frontRightAlignmentDisplayRad.setDouble(m_dDrivetrain.getFrontRightAngle() - frontRightInitialAngle);
        // Back left
        backLeftAngleDisplay.setDouble(m_dDrivetrain.getBackLeftAngle());
        backLeftRawAngleDisplay.setDouble(m_dDrivetrain.getBackLeftAngle() - DriveConstants.kBackLeftOffset);
        backLeftOffsetDisplay.setDouble(DriveConstants.kBackLeftOffset);
        backLeftAlignmentDisplayDeg.setDouble(Math.toDegrees(m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle));
        backLeftAlignmentDisplayRad.setDouble(m_dDrivetrain.getBackLeftAngle() - backLeftInitialAngle);
        // Back right
        backRightAngleDisplay.setDouble(m_dDrivetrain.getBackRightAngle());
        backRightRawAngleDisplay.setDouble(m_dDrivetrain.getBackRightAngle() - DriveConstants.kBackRightOffset);
        backRightOffsetDisplay.setDouble(DriveConstants.kBackRightOffset);
        backRightAlignmentDisplayDeg
                .setDouble(Math.toDegrees(m_dDrivetrain.getBackRightAngle() - backRightInitialAngle));
        backRightAlignmentDisplayRad.setDouble(m_dDrivetrain.getBackRightAngle() - backRightInitialAngle);
    }
}

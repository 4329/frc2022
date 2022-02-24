package frc.robot.Utilities;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretAngle extends SubsystemBase {
    double angle, radius;

    public TurretAngle() {

    }

    // Return a double between 0 and 360; left=180, right=0, forward=90
    public double getAngle(double x, double y) {
        // Joystick Deadzone
        if (x == 0 && y == 0) {
            return angle;
        }

        // Middle
        if (x == 0) {
            if (y > 0) {
                angle = 90;
            }
            if (y < 0) {
                angle = 270;
            }
        }

        // Right Half
        if (x > 0) {
            angle = Math.atan(y / x); // Find the angle
            angle = Math.toDegrees(angle);
        }

        // Left Half
        if (x < 0) {
            angle = Math.atan(y / x); // Find the angle
            angle = Math.toDegrees(angle);
            angle = angle + 180;
        }

        angle = resolveAngle(angle);

        return angle;
    }

    public double getRadius(double x, double y) {
        radius = Math.sqrt((x * x) + (y * y)); // Pythag
        return radius;
    }

    public static double resolveAngle(double angle) {
        while (angle > 360) {
            angle = angle - 360;
        }
        while (angle < 0) {
            angle = angle + 360;
        }
        return angle;
    }

}
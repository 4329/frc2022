package frc.robot.Subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.math.controller.PIDController;

@RunWith(MockitoJUnitRunner.class)
public class PidTests {
    // i need negative power to move right, and positive power to move left. 
    
    @Test
    public void limlightPid_txNegative_rotatesPositive() {
        PIDController pid = new PIDController(1, 0, 0);
        double output = pid.calculate(-17.6, 0);
        assertEquals(17.6, output, .1);
    }

    @Test
    public void limlight_txPositive_rotatesPositive() {
        PIDController pid = new PIDController(1, 0, 0);
        double output = pid.calculate(17.6, 0);
        assertEquals(-17.6, output, .1);
    }


    // if it rotates clockwise, increase in encoder.
    @Test
    public void zeroingTurret_underZero_rotatePositive() {
        PIDController pid = new PIDController(2, 0, 0);
        double output = pid.calculate(1000, 1250);
        assertEquals(500.0, output, .1); // problem - this is positve.  positve power will move me left.
    }

    @Test
    public void zeroingTurret_overZero_rotateNegative() {
        PIDController pid = new PIDController(2, 0, 0);
        double output = pid.calculate(1500, 1250);
        assertEquals(-500.0, output, .1); // problem - this is negative.  positve power will move me .
    }
}

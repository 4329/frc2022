package frc.robot.Subsystems.Swerve;

import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;

@RunWith(MockitoJUnitRunner.class)
public class IntakeSolenoidSubsystemTest {
   @Mock
   private Solenoid mockSolenoid;

   @Mock
   private PneumaticHub mockPneumaticHub;
   
   private IntakeSolenoidSubsystem testObject;

   @Before
   public void setUp() {
       when(mockPneumaticHub.makeSolenoid(anyInt())).thenReturn(mockSolenoid);
       this.testObject = new IntakeSolenoidSubsystem(mockPneumaticHub);
   }
    
   @Test
   public void intakeUp_raisesIntake() {
       testObject.intakeUp();

       verify(mockSolenoid).set(false);
   }

   @Test
   public void intakeDown_lowersIntake() {
       testObject.intakeDown();

       verify(mockSolenoid).set(true);
   }   

   @Test
   public void keepIntakePosition_solenoidTrue_intakeDownCalled() {
       when(mockSolenoid.get()).thenReturn(true);

       testObject.keepIntakePosition();

       verify(mockSolenoid).set(true);
   }

   @Test
   public void keepIntakePosition_solenoidFalse_intakeUpCalled() {
       when(mockSolenoid.get()).thenReturn(false);

       testObject.keepIntakePosition();

       verify(mockSolenoid).set(false);
   }
}

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
      when(mockPneumaticHub.makeSolenoid(0)).thenReturn(mockSolenoid);
      testObject = new IntakeSolenoidSubsystem(mockPneumaticHub);
   }

   @Test
   public void intakeUp_solenoidSet() {
      testObject.intakeUp();

      verify(mockSolenoid).set(false);
   }

   @Test
   public void intakeDown() {
      testObject.intakeDown();

      verify(mockSolenoid).set(true);
   }

   @Test
   public void keepIntakePosition_intakeDownGetsCalled() {
      when(mockSolenoid.get()).thenReturn(true);

      testObject.keepIntakePosition();

      verify(mockSolenoid).set(true);
   }

   @Test
   public void keepIntakePosition_intakeUpGetsCalled() {
      when(mockSolenoid.get()).thenReturn(false);

      testObject.keepIntakePosition();

      verify(mockSolenoid).set(false);
   }
}

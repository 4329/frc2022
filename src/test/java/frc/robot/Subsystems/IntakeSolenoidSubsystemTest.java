package frc.robot.Subsystems;

import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.junit.MockitoJUnitRunner;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@RunWith(MockitoJUnitRunner.class)
public class IntakeSolenoidSubsystemTest {
   @Mock
   private DoubleSolenoid mockSolenoid;

   @Mock
   private PneumaticHub mockPneumaticHub;
   
   private IntakeSolenoidSubsystem testObject;

   @Before
   public void setUp() {
      when(mockPneumaticHub.makeDoubleSolenoid(0, 1)).thenReturn(mockSolenoid);
      testObject = new IntakeSolenoidSubsystem(mockPneumaticHub);
   }

   @Test
   public void intakeUp_solenoidSet() {
      testObject.intakeUp();

      verify(mockSolenoid).set(Value.kReverse);
   }

   @Test
   public void intakeDown() {
      testObject.intakeDown();

      verify(mockSolenoid).set(Value.kForward);
   }
}

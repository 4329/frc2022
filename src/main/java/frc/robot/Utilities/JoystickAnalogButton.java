package frc.robot.Utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.*;

public class JoystickAnalogButton extends Button {

  XboxController m_controller;
  Hand m_hand;
  private double m_threshold = 0.25;

  /**
   * Create a button for triggering commands off a controller's analog axis
   * 
   * @param controller The controller to use
   * @param hand Which side of the controller
   */
  public JoystickAnalogButton(XboxController controller, Hand hand) {
      m_controller = controller;
      m_hand = hand;
  }

  public boolean get() {
      return m_controller.getTriggerAxis(m_hand) > m_threshold; 
  }

}
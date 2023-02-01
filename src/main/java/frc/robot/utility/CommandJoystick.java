package frc.robot.utility;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandJoystick extends Joystick {
  public Trigger frontTrigger = createTriggerFromJoystickButton(1);
  public Trigger two = createTriggerFromJoystickButton(2);
  public Trigger three = createTriggerFromJoystickButton(3);
  public Trigger four = createTriggerFromJoystickButton(4);
  public Trigger five = createTriggerFromJoystickButton(5);
  public Trigger six = createTriggerFromJoystickButton(6);
  public Trigger seven = createTriggerFromJoystickButton(7);
  public Trigger eight = createTriggerFromJoystickButton(8);
  public Trigger nine = createTriggerFromJoystickButton(9);
  public Trigger ten = createTriggerFromJoystickButton(10);
  public Trigger eleven = createTriggerFromJoystickButton(11);
  public Trigger twelve = createTriggerFromJoystickButton(12);

  public CommandJoystick(int portNumber) {
    super(portNumber);
  }

  private Trigger createTriggerFromJoystickButton(int buttonNumber) {
    return new Trigger(() -> this.getRawButtonPressed(buttonNumber));
  }
}

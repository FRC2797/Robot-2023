package frc.robot;

import frc.robot.subsystems.Lift;
import frc.robot.utility.CommandJoystick;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class RobotContainerForTesting {
  Lift lift = new Lift();
  CommandJoystick joystick = new CommandJoystick(0);
  public RobotContainerForTesting() {
    joystick.frontTrigger.whileTrue(startEnd(() -> lift.setSpeed(0.1), () -> lift.setSpeed(0), lift));
  }
}

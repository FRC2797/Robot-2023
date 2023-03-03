package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static frc.robot.InlineCommands.*;

public class RobotContainer {
  private final static CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.y().onTrue(setTelescopicArmPosition(0.5));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.InlineCommands.*;

public class RobotContainer {
  private final boolean IS_SEMI_AUTONOMOUS = false;
  CommandScheduler commandScheduler = CommandScheduler.getInstance();


  public RobotContainer() {
    configureBindings(IS_SEMI_AUTONOMOUS);
  }

  CommandXboxController c = controller;
  private void configureBindings(boolean isSemiAutonomous) {
    commandScheduler.getActiveButtonLoop().clear();
    drivetrain.setDefaultCommand(teleopDrive());

    c.b().onTrue(runOnce(() -> commandScheduler.cancelAll()));

    if (isSemiAutonomous)
      bindSemiAutonomous();
    else
      bindManualButtons();
  }

  private void bindManualButtons() {
    c.x().whileTrue(grabberOpen());

    c.y().whileTrue(liftUp());
    c.a().whileTrue(liftDown());

    c.povUp().whileTrue(telescopeForward());
    c.povDown().whileTrue(telescopeBackward());
  }

  private void bindSemiAutonomous() {
    // We won't need different lift heights for
    // pegs and shelves. We'll just lift to the
    // height of the pegs and the cube can just
    // drop down
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

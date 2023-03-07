package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.InlineCommands.*;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class RobotContainer {
  private final boolean IS_SEMI_AUTONOMOUS = false;
  CommandScheduler commandScheduler = CommandScheduler.getInstance();


  public RobotContainer() {
    configureBindings(IS_SEMI_AUTONOMOUS);
  }

  CommandXboxController c = controller;
  private void configureBindings(boolean isSemiAutonomous) {
    commandScheduler.getActiveButtonLoop().clear();
    drivetrain.setDefaultCommand(teleopDriveTankDrive());

    CommandBase cancelAllCommands = runOnce(() -> commandScheduler.cancelAll());
    // bind cancel all commands
    // usbButton.onTrue(cancelAllCommands);

    if (isSemiAutonomous)
      bindSemiAutonomous();
    else
      bindManualButtons();
  }

  private void bindManualButtons() {
    c.x().whileTrue(keepGrabberOpen());

    c.y().whileTrue(liftUp());
    c.a().whileTrue(liftDown());

    c.b().whileTrue(brakesOn());

    c.povUp().whileTrue(telescopeForward());
    c.povDown().whileTrue(telescopeBackward());
  }

  private void bindSemiAutonomous() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}

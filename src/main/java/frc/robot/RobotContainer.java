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
    c.x().whileTrue(grabberOpen());

    c.y().whileTrue(liftUp());
    c.a().whileTrue(liftDown());

    c.b().whileTrue(brakesOn());

    c.povUp().whileTrue(telescopeForward());
    c.povDown().whileTrue(telescopeBackward());
  }

  private void bindSemiAutonomous() {
    // We won't need different lift heights for
    // pegs and shelves. We'll just lift to the
    // height of the pegs and the cube can just
    // drop down

    // It'll also be good enough to just always aim at the lower peg
    // If there it isn't available it'll automatically
    // aim at the top peg

    bindSemiAutoLiftCommands(c.leftBumper(), InlineCommands::aimAprilTag);
    // TODO: When going for the middle it should aim for lower peg and
    // going for the top it should aim for the top peg
    bindSemiAutoLiftCommands(c.rightBumper(), InlineCommands::aimLowerPeg);

    c.rightTrigger().whileTrue(grabberOpen());
  }

  private void bindSemiAutoLiftCommands(Trigger bumper, Supplier<Command> aim) {
    bumper.and(c.povUp()).onTrue(sequence(aim.get(), liftToTop(), grabber.fullyOpen()));
    bumper.and(c.povLeft().or(c.povRight())).onTrue(sequence(aim.get(), liftToMiddle(), grabber.fullyOpen()));
    bumper.and(c.povDown()).onTrue(sequence(aim.get(), liftToBottom(), grabber.fullyOpen()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

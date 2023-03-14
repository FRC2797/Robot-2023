package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.InlineCommands.*;
import static frc.robot.Test.*;

public class RobotContainer {
  private final boolean IS_SEMI_AUTONOMOUS = true;
  CommandScheduler commandScheduler = CommandScheduler.getInstance();


  public RobotContainer() {
    configureBindings(IS_SEMI_AUTONOMOUS);
    addTestsToShuffleboard();
  }

  CommandXboxController c = controller;
  private void configureBindings(boolean isSemiAutonomous) {
    clearAllButtonBindings();
    drivetrain.setDefaultCommand(teleopDriveArcadeDrive());

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

    c.b().toggleOnTrue(brakesOn());

    c.povUp().whileTrue(telescopeForward());
    c.povDown().whileTrue(telescopeBackward());
  }

  private void bindSemiAutonomous() {
    Trigger startButton = c.start();
    Trigger backButton = c.back();

    CommandBase brakesOn = brakesOn();
    startButton.onTrue(runOnce(() -> brakesOn.schedule()));
    backButton.onTrue(runOnce(() -> brakesOn.cancel()));

    c.povUp().whileTrue(liftUp());
    c.povDown().whileTrue(liftDown());

    c.povRight().whileTrue(telescopeForward());
    c.povLeft().whileTrue(telescopeBackward());

    Trigger leftAndRightBumperNotPressed = (c.leftBumper().or(c.rightBumper())).negate();
    c.y().and(leftAndRightBumperNotPressed).toggleOnTrue(
      sequence(aimTopPeg(), liftToTop(), extensionForTop())
    );

    c.b().and(leftAndRightBumperNotPressed).toggleOnTrue(
      sequence(aimLowerPeg(), liftToMiddle(), extensionForMiddle())
    );

    c.x().and(leftAndRightBumperNotPressed).toggleOnTrue(
      sequence(aimAprilTag(), liftToTop(), extensionForTop())
    );

    c.a().and(leftAndRightBumperNotPressed).toggleOnTrue(
      sequence(aimAprilTag(), liftToMiddle(), extensionForMiddle())
    );

    c.y().and(c.leftBumper()).toggleOnTrue(aimTopPeg());
    c.y().and(c.rightBumper()).toggleOnTrue(liftToTop());

    c.x().and(c.leftBumper()).toggleOnTrue(aimAprilTag());
    c.x().and(c.rightBumper()).toggleOnTrue(liftToTop());

    c.a().and(c.leftBumper()).toggleOnTrue(aimAprilTag());
    c.a().and(c.rightBumper()).toggleOnTrue(liftToMiddle());

    c.b().and(c.rightBumper()).toggleOnTrue(liftToMiddle());
    c.b().and(c.leftBumper()).toggleOnTrue(aimLowerPeg());

    // Three different seeking
    CommandBase extendBackInAfterGrabbing = extensionBackIn().withName("Extend back in after grabbing");
    CommandBase extensionToGrabWhileGrabbingGamepiece = extensionToGrab().withName("extensionToGrabWhileGrabbingGamepiece");
    CommandBase grabGamepiece = parallel(
      new ScheduleCommand(extensionToGrabWhileGrabbingGamepiece),
      keepGrabberOpen()
    );

    c.rightTrigger().whileTrue(grabGamepiece);
    c.rightTrigger().onFalse(extendBackInAfterGrabbing);

    Trigger hasGamepiece = new Trigger(() -> grabber.hasGamepiece());
    c.leftTrigger().and(hasGamepiece)
      .toggleOnTrue(
        sequence(
          dropGamepiece(),
          extensionBackIn()
        )
    );

    CommandBase goIntoSeeking = sequence(extensionBackIn(), liftToBottom());
    c.leftTrigger().and((hasGamepiece.negate())).toggleOnTrue(goIntoSeeking);
  }

  //TODO: Code the switching of autos with a selector on shuffleboard
  public Command getAutonomousCommand() {
    return Autos.chargingStation();
  }

  public void clearAllButtonBindings() {
    commandScheduler.getActiveButtonLoop().clear();
  }


}

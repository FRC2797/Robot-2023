package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.InlineCommands.*;
import static frc.robot.Test.*;

import java.util.function.Supplier;

public class RobotContainer {
  private final boolean IS_SEMI_AUTONOMOUS = true;
  private final SendableChooser<CommandBase> autoChooser = new SendableChooser<>();
  CommandScheduler commandScheduler = CommandScheduler.getInstance();


  public RobotContainer() {
    configureBindings(IS_SEMI_AUTONOMOUS);
    addTestsToShuffleboard();
    configureAutoChooser();
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

    c.povUpRight().whileTrue(liftUp().deadlineWith(telescopeForward()));
    c.povUpLeft().whileTrue(liftUp().deadlineWith(telescopeBackward()));
    c.povDownRight().whileTrue(liftDown().deadlineWith(telescopeForward()));
    c.povDownLeft().whileTrue(liftDown().deadlineWith(telescopeBackward()));

    c.y().toggleOnTrue(liftToTop().andThen(keepLiftingUp().deadlineWith(extendForTop())));
    c.b().toggleOnTrue(liftToTop().andThen(keepLiftingUp().deadlineWith(extendForMiddle())));
    c.a().toggleOnTrue(extensionBackIn().andThen(liftToBottom()));
    c.x().onTrue(dropGamepiece());

    c.rightTrigger().whileTrue(keepGrabberOpen());
  }

  //TODO: Code the switching of autos with a selector on shuffleboard
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void clearAllButtonBindings() {
    commandScheduler.getActiveButtonLoop().clear();
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Place gamepiece at top", Autos.placeGamepiece.get());
    Shuffleboard.getTab("Driver").add(autoChooser);
  }


  private CommandBase grabGamepiece(double liftHeight, double extension) {
    return sequence(
      race(
        keepGrabberOpen(),
        liftToPosition(liftHeight)
      ),
      new ScheduleCommand(
        telescopeArm.setPositionCommand(extension).withName("Grab gamepiece set position command")
      ),
      keepGrabberOpen()
    );
  }
}


package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import frc.robot.subsystems.Navx;

public class RobotContainer {

  private final CommandXboxController controller =
      new CommandXboxController(0);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Navx navx = new Navx();

  public RobotContainer() {
    drivetrain.setDefaultCommand(teleopDrive());
    configureBindings();
  }

  private void configureBindings() {
    controller.y().onTrue(driveUntilLevelOnChargingStation());
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command teleopDrive() {
    return run(() -> {
      double leftY = controller.getLeftY();
      double rightX = controller.getRightX();

      drivetrain.arcadeDrive(
        transformStickInput(leftY), 
        transformStickInput(rightX)
      );
    }, drivetrain);
  }

  public double transformStickInput(double stickInput) {
    final double DEADBAND = 0.05;
    stickInput = applyDeadband(stickInput, DEADBAND);
    stickInput *= -1;
    boolean isNegative = stickInput < 0;
    stickInput *= stickInput;
    stickInput = isNegative ? -stickInput : stickInput;
    return stickInput;
  }

  public Command driveUntilLevelOnChargingStation() {
    final double SLOW_SPEED_FORWARD = 0.15;
    final double SLOW_SPEED_BACKWARD = -0.05;
    final double WAIT_BEFORE_OVERSHOOT_CORRECTION = 0.5;
    final double PITCHED_VALUE = 15;


    Command driveForwardSlowly = run(() -> drivetrain.arcadeDrive(SLOW_SPEED_FORWARD, 0), drivetrain);
    Command waitUntilPitched = waitUntil(() -> Math.abs(navx.getRoll()) > PITCHED_VALUE);

    Command driveBackwardSlowly = run(() -> drivetrain.arcadeDrive(SLOW_SPEED_BACKWARD, 0), drivetrain);

    return (
      deadline(waitUntilPitched.andThen(waitUntilLevel()), driveForwardSlowly)
      .andThen(deadline(waitSeconds(WAIT_BEFORE_OVERSHOOT_CORRECTION).andThen(waitUntilLevel()), driveBackwardSlowly))
    );
  }

  public Command waitUntilLevel() {
    final double LEVELED_VALUE = 1;

    return waitUntil(() -> Math.abs(navx.getRoll()) < LEVELED_VALUE);
  }
}

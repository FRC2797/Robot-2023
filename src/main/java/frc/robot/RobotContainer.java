
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.math.MathUtil.applyDeadband;

public class RobotContainer {

  private final CommandXboxController controller =
      new CommandXboxController(0);

  private final Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {
    drivetrain.setDefaultCommand(teleopDrive());
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command teleopDrive() {
    return run(() -> {
      final double DEADBAND = 0.1;


      double leftY = controller.getLeftY();
      double rightX = controller.getRightX();

      double leftYDeadband = applyDeadband(leftY, DEADBAND);
      double rightXDeadband = applyDeadband(rightX, DEADBAND);

      drivetrain.arcadeDrive(-leftYDeadband, rightXDeadband);
    }, drivetrain);
  }
}

package frc.robot;

import frc.robot.subsystems.Lift;
import frc.robot.utility.CommandJoystick;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainerForTesting {
  Lift lift = new Lift();
  CommandJoystick joystick = new CommandJoystick(0);
  public RobotContainerForTesting() {
    joystick.frontTrigger.onTrue(liftToPosition(Math.PI));
  }

  private CommandBase liftToPosition(double positionRadians) {
    final double kS = 0.1593;
    final double kV = 0.19465;
    final double kA = 0.010118;
    final double kG = 0.0708;

    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);


    final double kP_FACTOR_ADJUSTMENT = 0.1;
    final double kP = 5.7517 * kP_FACTOR_ADJUSTMENT;
    final double kD = 0.29588;

    PIDController pid = new PIDController(kP, 0.1, kD);
    Shuffleboard.getTab("Lift").add(pid);

    return run(() -> {
      double voltage = feedforward.calculate(positionRadians, 1)
        + pid.calculate(lift.getAngleInRadians(), positionRadians);

      lift.setVoltage(voltage);
    }, lift)
    .beforeStarting(() -> pid.reset())
    .finallyDo(end -> {
      lift.setVoltage(0);
      lift.setSpeed(0);
    });
  }
}

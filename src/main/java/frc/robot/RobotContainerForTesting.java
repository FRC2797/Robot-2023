package frc.robot;

import frc.robot.subsystems.Lift;
import frc.robot.utility.CommandJoystick;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainerForTesting {
  Lift lift = new Lift();
  CommandXboxController controller = new CommandXboxController(0);
  public RobotContainerForTesting() {
    final double SPEED = 1;
    controller.y().whileTrue(startEnd(() -> lift.setSpeed(SPEED), () -> lift.setSpeed(0), lift));
  }

  private CommandBase liftToPosition(double positionRadians) {
    final double kS = 0.039003;
    final double kV = 0.42996;
    final double kA = 0.028221;
    final double kG = 0.10426;

    ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);


    final double kP = 0;
    final double kI = 0;
    final double kD = 0;

    PIDController pid = new PIDController(kP, kI, kD);
    Shuffleboard.getTab("Lift").add(pid);

    return run(() -> {
      double voltage = feedforward.calculate(positionRadians, 0)
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

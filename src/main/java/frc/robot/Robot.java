package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static edu.wpi.first.math.MathUtil.interpolate;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {
  private final CANSparkMax grabberMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final double MAX_MOTOR_SPEED = 0.41;
  XboxController controller = new XboxController(0);

  @Override
  public void teleopInit() {
    grabberMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    double interpolatedMotorSpeed = interpolate(0, MAX_MOTOR_SPEED, controller.getRightTriggerAxis());
    grabberMotor.set(interpolatedMotorSpeed);
  }
}

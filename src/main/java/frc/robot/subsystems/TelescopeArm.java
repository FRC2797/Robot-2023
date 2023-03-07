package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static java.lang.Math.signum;
import static java.lang.Math.abs;

public class TelescopeArm extends SubsystemBase {
  private final int MOTOR_ID = 2;
  private final CANSparkMax motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  public TelescopeArm() {
    final boolean IS_INVERTED = true;
    motor.setInverted(IS_INVERTED);

    final double UNITS_AT_FULL_EXTENSION = 42.8;
    encoder.setPositionConversionFactor(1 / UNITS_AT_FULL_EXTENSION);
    encoder.setPosition(0);

    Shuffleboard.getTab("Telescope Arm").addDouble("Encoder Position", encoder::getPosition);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getPercentageExtended() {
    return encoder.getPosition();
  }

  public boolean isFullyExtended() {
    return getPercentageExtended() > 0.9;
  }

  public boolean isFullyIn() {
    return getPercentageExtended() < 0.1;
  }

  public CommandBase setPositionCommand(double percentageSetpoint) {
    final double PROPORTIONAL_TERM = 0.5;
    final double MIN_TERM = 0.05;
    final double TOLERANCE = 0.03;

    return run(() -> {
      double error = percentageSetpoint - getPercentageExtended();
      double speed = (error * PROPORTIONAL_TERM) + (signum(error) * MIN_TERM);
      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("Speed", speed);

      setSpeed(speed);
    })
    .until(() -> abs(getPercentageExtended()) < TOLERANCE)
    .finallyDo(end -> setSpeed(0));
  }
}

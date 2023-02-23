package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  final private CANSparkMax motor;
  final private RelativeEncoder encoder;

  public Lift() {
    final int MOTOR_ID = 2;
    final boolean IS_INVERTED = true;
    final boolean IS_BRAKED = true;

    motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(IS_INVERTED);
    motor.setIdleMode(IS_BRAKED ? IdleMode.kBrake : IdleMode.kCoast);

    encoder = motor.getEncoder();
    final double ENCODER_COUNT_IN_A_SINGLE_ROTATION = 17.4;
    encoder.setPositionConversionFactor(1 / ENCODER_COUNT_IN_A_SINGLE_ROTATION);
    encoder.setPosition(0);

    configureShuffleboard();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
 * @return angle in radians from the horizontal
 */
public double getAngleInRadians() {
    return encoder.getPosition() * 2 * Math.PI;
}

  private void configureShuffleboard() {
    ShuffleboardTab lift = Shuffleboard.getTab("Lift");
    lift.addDouble("Motor speed", motor::get);
    lift.addDouble("Encoder Position", encoder::getPosition);
    lift.addDouble("Get Angle in Radians", this::getAngleInRadians);
  }
}

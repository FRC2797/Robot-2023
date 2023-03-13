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
    final int MOTOR_ID = 5;
    final boolean IS_INVERTED = false;
    final boolean IS_BRAKED = true;

    motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(IS_INVERTED);
    motor.setIdleMode(IS_BRAKED ? IdleMode.kBrake : IdleMode.kCoast);

    encoder = motor.getEncoder();
    final double ENCODER_COUNTS_FOR_HIGHEST_ROTATION = 42;
    encoder.setPositionConversionFactor(1 / ENCODER_COUNTS_FOR_HIGHEST_ROTATION);
    resetEncoder();

    configureShuffleboard();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public double getPercentageOfHighestRotation() {
    return encoder.getPosition();
  }

  public boolean isFullyUp() {
    return getPercentageOfHighestRotation() > 1;
  }

  public boolean isFullyDown() {
    return getPercentageOfHighestRotation() < 0.05;
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  private void configureShuffleboard() {
    ShuffleboardTab lift = Shuffleboard.getTab("Lift");
    lift.addDouble("Motor speed", motor::get);
    lift.addDouble("Encoder Position", encoder::getPosition);
    lift.addBoolean("Is fully up", this::isFullyUp);
    lift.addBoolean("Is fully down", this::isFullyDown);
  }
}

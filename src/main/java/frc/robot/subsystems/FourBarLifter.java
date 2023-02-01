package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarLifter extends SubsystemBase {
  final private int MOTOR_ID = 8;
  private CANSparkMax motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder encoder;

  public FourBarLifter() {
    final boolean MOTOR_IS_INVERTED = true;
    motor.setInverted(MOTOR_IS_INVERTED);
    configureEncoder();
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed);
  }

  private void configureEncoder() {
    final double SET_TO_EXTENSION_PERCENTAGE = 1;
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(SET_TO_EXTENSION_PERCENTAGE);
    encoder.setPosition(0);
  }

  private double getPercentageUp() {
    return encoder.getPosition();
  }
}


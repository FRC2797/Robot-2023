package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarLifter extends SubsystemBase {
  final private int MOTOR_ID = 6;
  private CANSparkMax motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder encoder;

  public FourBarLifter() {
    final boolean MOTOR_IS_INVERTED = true;
    motor.setInverted(MOTOR_IS_INVERTED);
    configureEncoder();
    setUpShuffleboard();
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

  private void setUpShuffleboard() {
    ShuffleboardTab fourBarLift = Shuffleboard.getTab("Four Bar Lifter");
    fourBarLift.addDouble("Get percentage up", this::getPercentageUp);
    fourBarLift.addDouble("Motor Power", motor::get);
  }

  private double getPercentageUp() {
    return encoder.getPosition();
  }
}


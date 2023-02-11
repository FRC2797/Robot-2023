package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  final private CANSparkMax motor;
  final private RelativeEncoder encoder;

  public Lift() {
    final int MOTOR_ID = 3;
    final boolean IS_INVERTED = true;
    motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(IS_INVERTED);

    encoder = motor.getEncoder();

    configureShuffleboard();
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  private void configureShuffleboard() {
    ShuffleboardTab lift = Shuffleboard.getTab("Lift");
    lift.addDouble("Motor speed", motor::get);
    lift.addDouble("Encoder", encoder::getPosition);
  }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeArm extends SubsystemBase {
  private final int MOTOR_ID = 3;
  CANSparkMax motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
  public TelescopeArm() {

  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}

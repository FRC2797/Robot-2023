package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final CANSparkMax grabberMotor = new CANSparkMax(5, MotorType.kBrushless);

  @Override
  public void teleopPeriodic() {
    grabberMotor.set(0.05);
  }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");

  public Grabber() {
    final boolean MOTOR_INVERSION = true;
    final int MOTOR_ID = 8;

    motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    motor.setInverted(MOTOR_INVERSION);
    motor.setIdleMode(IdleMode.kBrake);

    RelativeEncoder encoder = motor.getEncoder();
    final double COUNTS_FULLY_OPEN = 32;
    encoder.setPositionConversionFactor(1 / COUNTS_FULLY_OPEN);

    grabberTab.addBoolean("Is Open", this::isFullyOpen);
    grabberTab.add(this);
  }

  private final double OPENING_SPEED = 0.15;

  public CommandBase fullyOpen() {
    return open(OPENING_SPEED).until(this::isFullyOpen);
  }

  private final int OPEN_LIMIT_SWITCH_ID = 0;
  private final DigitalInput openLimitSwitch = new DigitalInput(OPEN_LIMIT_SWITCH_ID);

  private boolean isFullyOpen() {
    return !openLimitSwitch.get();
  }

  public CommandBase open(double speed) {
    CommandBase openCommand = startEnd(() -> motor.set(speed), () -> motor.set(0));
    openCommand.addRequirements(this);
    return openCommand;
  }

  public boolean hasGamepiece() {
    return getPercentageOpen() > 0.2;
  }

  public double getPercentageOpen() {
    return encoder.getPosition();
  }
}

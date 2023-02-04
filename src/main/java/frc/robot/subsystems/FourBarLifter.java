package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarLifter extends SubsystemBase {
  final private int LEFT_ID = 6;
  final private int RIGHT_ID = 6;
  private CANSparkMax leftMotor = new CANSparkMax(LEFT_ID, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(RIGHT_ID, MotorType.kBrushless);
  private MotorControllerGroup motors = new MotorControllerGroup(leftMotor, rightMotor);

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  public FourBarLifter() {
    final boolean MOTOR_IS_INVERTED = true;
    motors.setInverted(MOTOR_IS_INVERTED);
    configureEncoders();
    setUpShuffleboard();
  }

  public void setMotorSpeed(double speed) {
    motors.set(speed);
  }

  private void configureEncoders() {
    final double SET_TO_EXTENSION_PERCENTAGE = -1/2.54;
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(SET_TO_EXTENSION_PERCENTAGE);
    rightEncoder.setPositionConversionFactor(SET_TO_EXTENSION_PERCENTAGE);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  private void setUpShuffleboard() {
    ShuffleboardTab fourBarLift = Shuffleboard.getTab("Four Bar Lifter");
    fourBarLift.addDouble("Get percentage up", this::getPercentageUp);
    fourBarLift.addDouble("Motor Power", motors::get);
  }

  private double getPercentageUp() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
  }
}


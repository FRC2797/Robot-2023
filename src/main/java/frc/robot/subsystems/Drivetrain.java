package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrive drive;

  private CANSparkMax sparkMaxFrontRight;
  private CANSparkMax sparkMaxFrontLeft;
  private CANSparkMax sparkMaxBackRight;
  private CANSparkMax sparkMaxBackLeft;

  public Drivetrain() {
    final int FRONT_RIGHT = 5;
    final int BACK_RIGHT = 2;
    final int FRONT_LEFT = 3;
    final int BACK_LEFT = 4;

    sparkMaxFrontRight = new CANSparkMax(FRONT_RIGHT, MotorType.kBrushless);
    sparkMaxFrontLeft = new CANSparkMax(FRONT_LEFT, MotorType.kBrushless);
    sparkMaxBackRight = new CANSparkMax(BACK_RIGHT, MotorType.kBrushless);
    sparkMaxBackLeft = new CANSparkMax(BACK_LEFT, MotorType.kBrushless);

    MotorControllerGroup leftMotors = new MotorControllerGroup(sparkMaxFrontLeft, sparkMaxBackLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(sparkMaxFrontRight, sparkMaxBackRight);
    leftMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(0);
  }



  public void arcadeDrive(double xSpeed, double rotation) {
    final boolean INPUTS_SQUARED = false;
    drive.arcadeDrive(xSpeed, rotation, INPUTS_SQUARED);
  }
}

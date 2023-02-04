package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private DifferentialDrive drive;

  private CANSparkMax frontRight;
  private CANSparkMax frontLeft;
  private CANSparkMax backRight;
  private CANSparkMax backLeft;

  private RelativeEncoder frontRightEnc;
  private RelativeEncoder frontLeftEnc;
  private RelativeEncoder backRightEnc;
  private RelativeEncoder backLeftEnc;

  public Drivetrain() {
    final int FRONT_RIGHT = 1;
    final int BACK_RIGHT = 2;
    final int FRONT_LEFT = 7;
    final int BACK_LEFT = 4;

    frontRight = new CANSparkMax(FRONT_RIGHT, MotorType.kBrushless);
    frontLeft = new CANSparkMax(FRONT_LEFT, MotorType.kBrushless);
    backRight = new CANSparkMax(BACK_RIGHT, MotorType.kBrushless);
    backLeft = new CANSparkMax(BACK_LEFT, MotorType.kBrushless);

    MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, backRight);
    rightMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors);
    drive.setDeadband(0);

    frontRightEnc = frontRight.getEncoder();
    frontLeftEnc = frontLeft.getEncoder();
    backRightEnc = backRight.getEncoder();
    backLeftEnc = backLeft.getEncoder();

    final double OUTPUT_ROTATION_IN_INPUT_ROTATION = 1/8.19;

    frontRightEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    frontLeftEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    backRightEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    backLeftEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);

    setMotorsToBrake();
    resetEncoders();
    setUpShuffleboard();
  }


  private double getWheelRotations() {
      double total =
        frontLeftEnc.getPosition()
        + (frontRightEnc.getPosition() * -1)
        + backLeftEnc.getPosition()
        + (backRightEnc.getPosition() * -1);

      return total / 4;
  }

  public double getDistanceDrivenInInches() {
    final double WHEEL_DIAMETER = 8;
    return getWheelRotations() * WHEEL_DIAMETER * Math.PI;
  }



  public void arcadeDrive(double xSpeed, double rotation) {
    final boolean INPUTS_SQUARED = false;
    drive.arcadeDrive(xSpeed, rotation, INPUTS_SQUARED);
  }

  public void setMotorsToBrake() {
    frontRight.setIdleMode(IdleMode.kBrake);
    frontLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
  }

  private void setMotorsToCoast() {
    frontRight.setIdleMode(IdleMode.kCoast);
    frontLeft.setIdleMode(IdleMode.kCoast);
    backRight.setIdleMode(IdleMode.kCoast);
    backLeft.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    frontRightEnc.setPosition(0);
    frontLeftEnc.setPosition(0);
    backRightEnc.setPosition(0);
    backLeftEnc.setPosition(0);

  }

  private void setUpShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add(this);
    tab.add(drive);
    tab.addDouble("Distance drive in inches", this::getDistanceDrivenInInches);
    tab.addDouble("Get wheel rotations", this::getWheelRotations);
    tab.addDouble("Front Left Encoder get position", frontLeftEnc::getPosition);
    tab.addDouble("Front Right Encoder get position", frontRightEnc::getPosition);
    tab.addDouble("Back Left Encoder get position", backLeftEnc::getPosition);
    tab.addDouble("Back Right Encoder get position", backRightEnc::getPosition);
  }
}

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

    final double OUTPUT_ROTATION_IN_INPUT_ROTATION = 1;

    frontRightEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    frontLeftEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    backRightEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);
    backLeftEnc.setPositionConversionFactor(OUTPUT_ROTATION_IN_INPUT_ROTATION);

    setMotorsToBrake();
    setUpShuffleboard();
  }


  private double getWheelRotations() {
      return (frontLeftEnc.getPosition()
            + frontRightEnc.getPosition()
            + backLeftEnc.getPosition()
            + backRightEnc.getPosition())
        / 4;
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

  private void setUpShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    tab.add(this);
    tab.add(drive);
  }
}

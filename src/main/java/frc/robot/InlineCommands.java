package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.TelescopeArm;
import frc.robot.subsystems.Limelight.Pipeline;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import static edu.wpi.first.wpilibj2.command.Commands.*;

final public class InlineCommands {
  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Navx navx = new Navx();
  public final static Limelight limelight = new Limelight();
  public final static Grabber grabber = new Grabber();
  public final static TelescopeArm telescopeArm = new TelescopeArm();
  public final static Solenoid brakes = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public final static Lift lift = new Lift();

  public final static CommandXboxController controller = new CommandXboxController(0);

  private InlineCommands() {

  }

  public static CommandBase brakesOn() {
    return startEnd(() -> brakes.set(true), () -> brakes.set(false), drivetrain)
      .ignoringDisable(true);
  }

  private static final double DRIVE_POS_ACCEL_LIM_PER_SEC = 0.5;
  private static final double DRIVE_NEG_ACCEL_LIM_PER_SEC = -0.33;
  private static SlewRateLimiter forwardLimiter = new SlewRateLimiter(
    DRIVE_POS_ACCEL_LIM_PER_SEC,
    DRIVE_NEG_ACCEL_LIM_PER_SEC,
    0
  );

  private static SlewRateLimiter rotationLimiter = new SlewRateLimiter(
    DRIVE_POS_ACCEL_LIM_PER_SEC,
    DRIVE_NEG_ACCEL_LIM_PER_SEC,
    0
  );

  public static CommandBase teleopDriveArcadeDrive() {
    return run(
      () -> {
        double leftY = controller.getLeftY();
        double rightX = controller.getRightX();
        double transformedLeftY = transformStickInput(leftY);
        double transformedRightX = transformStickInput(rightX);

          drivetrain.arcadeDrive(
            forwardLimiter.calculate(transformedLeftY),
            transformedRightX
          );
        },
      drivetrain).withName("Teleop Arcade Drive");
  }

  public static CommandBase teleopDriveTankDrive() {
    return run(
      () -> {
        double leftY = controller.getLeftY();
        double rightY = controller.getRightY();

        drivetrain.tankDrive(transformStickInput(leftY), transformStickInput(rightY));
      },
      drivetrain);
  }

  private static double transformStickInput(double stickInput) {
    final double DEADBAND = 0.05;
    stickInput = applyDeadband(stickInput, DEADBAND);
    stickInput *= -1;
    boolean isNegative = stickInput < 0;
    stickInput *= stickInput;
    stickInput = isNegative ? -stickInput : stickInput;
    return stickInput;
  }

  public static CommandBase telescopeArmControl() {
    return run(
      () -> {
        telescopeArm.setSpeed(applyDeadband(controller.getLeftY(), 0.2));
      },
    telescopeArm);
  }



  private static CommandBase driveXCmd(double speed) {
    return run(() -> drivetrain.arcadeDrive(speed, 0), drivetrain);
  }


  private static CommandBase waitUntilLessThanPitch(double pitch) {
    return waitUntil(() -> Math.abs(navx.getPitch()) < pitch);
  }

  public static CommandBase driveUntilLevelOnChargingStation(double slowSpeed) {
    final double slowerSpeed = slowSpeed - 0.05 * (Math.signum(slowSpeed));
    final double WAIT_BEFORE_OVERSHOOT_CORRECTION = 0.5;
    final double PITCHED_VALUE = 12;

    Command waitUntilPitched = waitUntil(() -> Math.abs(navx.getPitch()) > PITCHED_VALUE);

    return
      (driveXCmd(slowSpeed).raceWith(waitUntil(() -> Math.abs(navx.getPitch()) > PITCHED_VALUE)))
      .andThen(driveXCmd(slowerSpeed).raceWith(waitUntilLessThanPitch(8)))
      .withName("Drive Until level on Charging station with " + slowSpeed + " speed");
  }

  public static CommandBase driveDistance(double inches) {
    final double PROP_TERM = 0.004;
    final double MIN_TERM = inches > 0 ? 0.05 : -0.05;
    final double distanceToDrive = inches;
    return runOnce(drivetrain::resetEncoders)
      .andThen(
        run(
          () -> {
            double distanceDriven = drivetrain.getDistanceDrivenInInches();
            double error = distanceToDrive - distanceDriven;
            double speed = (error * PROP_TERM) + MIN_TERM;

            drivetrain.arcadeDrive(speed, 0);
          },
          drivetrain)
            .until(() -> abs(drivetrain.getDistanceDrivenInInches()) > abs(inches)))
            .withName("Drive " + inches + " inches");
  }

  public static CommandBase driveRotation(double degrees) {
    final double PROP_TERM = 0.004;
    final double MIN_TERM = 0.1;
    final double TOLERANCE = 1;
    final double initialYaw = navx.getYaw();
    final double distanceToRotate = degrees;
    return run(
          () -> {
            double distanceRotated = navx.getYaw() - initialYaw;
            double error = distanceToRotate - distanceRotated;
            double speed = (error * PROP_TERM) + MIN_TERM * signum(error);

            drivetrain.arcadeDrive(0, speed);
          },
          drivetrain)
            .until(() -> abs(navx.getYaw() - initialYaw) < TOLERANCE)
            .withName("rotate " + degrees);
  }

  public static CommandBase waitUntilLevel() {
    final double LEVELED_VALUE = 3;

    return waitUntil(() -> Math.abs(navx.getPitch()) < LEVELED_VALUE);
  }

  public static CommandBase aimWithLimelight() {
    final double TOLERANCE = 1;
    final double SPEED = 0.2;
    return run(
      () -> {
        double horizontalOffset = limelight.getHorizontalOffset();

        if (horizontalOffset > TOLERANCE) {
          drivetrain.arcadeDrive(0, -SPEED);
        } else if (horizontalOffset < TOLERANCE) {
          drivetrain.arcadeDrive(0, SPEED);
        }
      },
      drivetrain
    )
    .until(() -> abs(limelight.getHorizontalOffset()) < TOLERANCE)
    .withName("aim with limelight");
  }

  public static CommandBase aimAprilTag() {
    return switchPipelineThenAim(Pipeline.aprilTag).withName("Aim April Tag");
  }

  public static CommandBase aimLowerPeg() {
    return switchPipelineThenAim(Pipeline.bottomPeg).withName("Aim Bottom Peg");
  }

  public static CommandBase aimTopPeg() {
    return switchPipelineThenAim(Pipeline.topPeg).withName("Aim Top Peg");
  }

  public static CommandBase liftToTop() {
    final double TOP_PERCENTAGE = 0.98;
    return liftToPosition(TOP_PERCENTAGE);
  }

  public static CommandBase liftToMiddle() {
    final double MIDDLE_PERCENTAGE = 0.5;
    return liftToPosition(MIDDLE_PERCENTAGE);
  }

  public static CommandBase liftToBottom() {
    CommandBase goDownUntilSwitchHit =
    run(() -> lift.setSpeed(-0.1), lift)
      .finallyDo(end -> lift.setSpeed(0))
      .until(lift::isFullyDown);
    final double BOTTOM_PERCENTAGE = 0;

    return liftToPosition(BOTTOM_PERCENTAGE).andThen(goDownUntilSwitchHit);
  }

  public static CommandBase extensionToGrab() {
    final double EXTENSION_PERCENTAGE = 0.25;
    return telescopeArm.setPositionCommand(EXTENSION_PERCENTAGE).withName("Extension to grab");
  }

  public static CommandBase extensionBackIn() {
    CommandBase extendInUntilSwitchHit =
    run(() -> telescopeArm.setSpeed(-0.2), telescopeArm)
      .finallyDo(end -> telescopeArm.setSpeed(0))
      .until(telescopeArm::isFullyIn);
    return telescopeArm.setPositionCommand(0).andThen(extendInUntilSwitchHit).withName("Extension back in");
  }

  public static CommandBase liftToPosition(double percentageOfHighestRotation) {
    final double PROP_TERM = 0.2;
    final double MIN_TERM = 0.1;
    final double TOLERANCE = 0.02;
    final double setpoint = percentageOfHighestRotation;
    return
      run(() -> {
        double currentHeight = lift.getPercentageOfHighestRotation();
        double error = setpoint - currentHeight;
        double speed = (error * PROP_TERM) + (MIN_TERM * signum(error));

        lift.setSpeed(speed);;
      }, lift)
      .finallyDo(end -> lift.setSpeed(0))
      .until(() -> {
        double currentHeight = lift.getPercentageOfHighestRotation();
        double error = setpoint - currentHeight;

        return abs(error) < abs(TOLERANCE);
      }).withName("Lift to " + percentageOfHighestRotation);
  }

  private static CommandBase switchPipelineThenAim(Pipeline pipeline) {
    Command switchPipeline = limelight.switchPipelineCommand(pipeline);
    return switchPipeline.andThen(aimWithLimelight());
  }

  public static CommandBase keepLiftingUp() {
    return run(() -> lift.setSpeed(0.2));
  }

  public static CommandBase extendForTop() {
    return telescopeArm.setPositionCommand(1);
  }

  public static CommandBase extendForMiddle() {
    return telescopeArm.setPositionCommand(0.19);
  }

  final private static double LIFT_SPEED = 0.20;
  public static CommandBase liftUp() {
    return run(() -> {
      if (true)
        lift.setSpeed(LIFT_SPEED);
      else
        lift.setSpeed(0);
    }, lift)
    .finallyDo(end -> lift.setSpeed(0));
  }

  public static CommandBase liftDown() {
    return run(() -> {
      if (!lift.isFullyDown())
        lift.setSpeed(-LIFT_SPEED);
      else
        lift.setSpeed(0);
    }, lift)
    .finallyDo(end -> lift.setSpeed(0));
  }

  final private static double TELESCOPE_SPEED = 0.2;
  public static CommandBase telescopeForward() {
    return run(() -> {
      if (!telescopeArm.isFullyExtended())
        telescopeArm.setSpeed(TELESCOPE_SPEED);
      else
        telescopeArm.setSpeed(0);
    }, telescopeArm)
    .finallyDo(end -> telescopeArm.setSpeed(0));
  }

  public static CommandBase telescopeBackward() {
    return run(() -> {
      if (!telescopeArm.isFullyIn())
        telescopeArm.setSpeed(-TELESCOPE_SPEED);
      else
        telescopeArm.setSpeed(0);
    }, telescopeArm)
    .finallyDo(end -> telescopeArm.setSpeed(0));
  }

  final public static CommandBase keepGrabberOpen() {
    final double GRABBER_SPEED = 0.4;
    return grabber.open(GRABBER_SPEED);
  }

  final public static CommandBase dropGamepiece() {
    return keepGrabberOpen().raceWith(waitSeconds(0.5)).withName("Drop gamepiece");
  }
}

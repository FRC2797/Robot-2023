
package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;
import frc.robot.subsystems.Grabber;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.math.MathUtil.applyDeadband;
import static java.lang.Math.abs;
import static frc.robot.subsystems.Limelight.*;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.TelescopeArm;

public class RobotContainer {

  private final CommandXboxController controller =
      new CommandXboxController(0);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Navx navx = new Navx();
  private final Limelight limelight = new Limelight();
  private final ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");
  private final Grabber grabber = new Grabber();
  private final TelescopeArm telescopeArm = new TelescopeArm();
  private final Solenoid brakes = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    CommandBase aimAprilTag = switchPipelineThenAim(Pipeline.aprilTag).withName("Aim April Tag");
    CommandBase aimBottomPeg = switchPipelineThenAim(Pipeline.bottomPeg).withName("Aim Bottom Peg");
    CommandBase aimTopPeg = switchPipelineThenAim(Pipeline.topPeg).withName("Aim Top Peg");
  public RobotContainer() {
    configureBindings();

  }

  private CommandBase brakesOn() {
    return startEnd(() -> brakes.set(true), () -> brakes.set(false), drivetrain).ignoringDisable(true);
  }

  CommandJoystick joystick = new CommandJoystick(0);
  private void configureBindings() {
    telescopeArm.setDefaultCommand(telescopeArmControl());
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private Command teleopDrive() {
    return run(() -> {
      double leftY = controller.getLeftY();
      double rightX = controller.getRightX();

      drivetrain.arcadeDrive(
        transformStickInput(leftY),
        transformStickInput(rightX)
      );
    }, drivetrain);
  }

  private CommandBase telescopeArmControl() {
    return run(() -> {
      telescopeArm.setSpeed(applyDeadband(-joystick.getX(), 0.2));
    }, telescopeArm);
  }

  private double transformStickInput(double stickInput) {
    final double DEADBAND = 0.05;
    stickInput = applyDeadband(stickInput, DEADBAND);
    stickInput *= -1;
    boolean isNegative = stickInput < 0;
    stickInput *= stickInput;
    stickInput = isNegative ? -stickInput : stickInput;
    return stickInput;
  }

  private Command driveUntilLevelOnChargingStation() {
    final double SLOW_SPEED_FORWARD = 0.15;
    final double SLOW_SPEED_BACKWARD = -0.05;
    final double WAIT_BEFORE_OVERSHOOT_CORRECTION = 0.5;
    final double PITCHED_VALUE = 15;


    Command driveForwardSlowly = run(() -> drivetrain.arcadeDrive(SLOW_SPEED_FORWARD, 0), drivetrain);
    Command waitUntilPitched = waitUntil(() -> Math.abs(navx.getPitch()) > PITCHED_VALUE);

    Command driveBackwardSlowly = run(() -> drivetrain.arcadeDrive(SLOW_SPEED_BACKWARD, 0), drivetrain);

    return (
      driveForwardSlowly.raceWith(waitUntilPitched.andThen(waitUntilLevel()))
      .andThen(
        driveBackwardSlowly.raceWith(waitSeconds(WAIT_BEFORE_OVERSHOOT_CORRECTION).andThen(waitUntilLevel()))
      )
    );
  }

  // TODO: Doesn't reset distance after each
  private Command driveDistance(double inches) {
    final double PROP_TERM = 0.00;
    final double MIN_TERM = 0.05;
    final double distanceToDrive = inches;
    return run(() -> {
      double distanceDriven = drivetrain.getDistanceDrivenInInches();
      double error = distanceToDrive - distanceDriven;
      double speed = (error * PROP_TERM) + MIN_TERM;

      drivetrain.arcadeDrive(speed, 0);
    }, drivetrain)
      .until(() -> drivetrain.getDistanceDrivenInInches() > inches);
  }

  private Command waitUntilLevel() {
    final double LEVELED_VALUE = 1;

    return waitUntil(() -> Math.abs(navx.getPitch()) < LEVELED_VALUE);
  }

  private CommandBase aimWithLimelight() {
    final double LINED_UP = 1;
    final double SPEED = 0.04;
    return run(() -> {
      double horizontalOffset = limelight.getHorizontalOffset();

      if (horizontalOffset > LINED_UP) {
        drivetrain.arcadeDrive(0, -SPEED);
      } else if (horizontalOffset < LINED_UP) {
        drivetrain.arcadeDrive(0, SPEED);
      }

    }, drivetrain).until(() -> abs(limelight.getHorizontalOffset()) < LINED_UP).withName("aim with limelight");
  }

  private CommandBase switchPipelineThenAim(Pipeline pipeline) {
    Command switchPipeline = limelight.switchPipelineCommand(pipeline);
    return switchPipeline.andThen(aimWithLimelight());
  }
}

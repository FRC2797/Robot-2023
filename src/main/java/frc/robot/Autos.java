package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.InlineCommands.*;

import java.util.function.Supplier;

public class Autos {
  private Autos() {

  }

  public static Supplier<CommandBase> placeGamepiece =
    () ->
    liftToTop()
    .andThen(
      liftToTop().repeatedly()
      .raceWith(
        sequence(
          extendForTop(),
          dropGamepiece(),
          extensionBackIn()
        )
      )
    )
    .andThen(liftToBottom())
    .withName("Placing a game piece");

  public static CommandBase chargingStation() {
    final double CHARGING_STATION_SPEED = -0.15;

    return sequence(
      placeGamepiece.get(),
      driveUntilLevelOnChargingStation(CHARGING_STATION_SPEED)
    )
    .withName("Charging Station Auto");
  }

  public static CommandBase twoGamepieceAuto() {
    final double DISTANCE_TO_GAMEPIECE = 210;
    return sequence(
      placeGamepiece.get(),
      driveRotation(180),
      driveDistance(DISTANCE_TO_GAMEPIECE),
      liftToBottom(),
      race(
        extensionToGrab(),
        keepGrabberOpen()
      ).finallyDo(end -> extensionBackIn()),
      driveRotation(180),
      driveDistance(DISTANCE_TO_GAMEPIECE),
      placeGamepiece.get()
    )
    .withName("Two Gamepiece Auto");
  }
}

package frc.robot;
import static frc.robot.Autos.*;
import static frc.robot.InlineCommands.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Test {
    // private CommandBase test1()
    // private CommandBase test2()
    // ...

    /*  public void addTestsToShuffleboard() {
        testTab.add(test1());
        testTab.add(test2());
    }
    */

    private static ShuffleboardTab testTab = Shuffleboard.getTab("Test");
    public static void addTestsToShuffleboard()
    {

        addCommandsToShuffleboard(
            driveRotation(90),
            driveRotation(180),
            driveRotation(360),
            driveDistance(40),
            driveDistance(120),
            driveUntilLevelOnChargingStation(-0.15),
            liftToTop(),
            liftToMiddle(),
            liftToBottom(),
            extensionBackIn(),
            extensionForBottom(),
            extensionForMiddle(),
            extensionForTop(),
            extensionToGrab(),
            aimAprilTag(),
            aimLowerPeg(),
            aimTopPeg(),
            twoGamepieceAuto(),
            chargingStation(),
            placeGamepiece.get(),
            liftToTop().andThen(extensionForTop()).withName("Lift to top then extension for top"),
            liftToTop().repeatedly().withName("Keep lifting to top"),
            liftToMiddle().repeatedly().withName("Keep lifting to middle"),
            dropGamepiece(),
            keepGrabberOpen()
        );
    }

    private static void addCommandsToShuffleboard(CommandBase... commands) {
        for (CommandBase cmd : commands) {
            testTab.add(cmd);
        }
    }

}

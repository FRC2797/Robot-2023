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
            liftToTop(),
            liftToMiddle(),
            liftToBottom(),
            extensionForBottom(),
            extensionForMiddle(),
            extensionForTop(),
            aimAprilTag(),
            aimLowerPeg(),
            aimTopPeg(),
            twoGamepieceAuto(),
            chargingStation()
        );
    }

    private static void addCommandsToShuffleboard(CommandBase... commands) {
        for (CommandBase cmd : commands) {
            testTab.add(cmd);
        }
    }

}

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.InlineCommands.*;

public class RobotContainer {


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // We won't need different lift heights for
    // pegs and shelves. We'll just lift to the
    // height of the pegs and the cube can just
    // drop down
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

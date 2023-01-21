package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Limelight extends SubsystemBase {
  private final int ENTRY_NOT_FOUND = -9999;

  public enum Pipeline {
    bottomPeg(0),
    topPeg(1),
    aprilTag(2);

    @SuppressWarnings("MemberName")
    public final int value;

    Pipeline(int value) {
      this.value = value;
    }
  }

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


  public double getHorizontalOffset() {
    NetworkTableEntry horizontalOffset = table.getEntry("tx");
    return horizontalOffset.getDouble(ENTRY_NOT_FOUND);
  }

  private void switchPipeline(Pipeline pipeline) {
    table.getEntry("pipeline").setNumber(pipeline.value);
  }

  public CommandBase switchPipelineCommand(Pipeline pipeline) {
    return runOnce(() -> switchPipeline(pipeline)).andThen(waitSeconds(0.1));
  }
}

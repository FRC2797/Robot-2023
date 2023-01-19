package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final int ENTRY_NOT_FOUND = -9999;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double getHorizontalOffset() {
    NetworkTableEntry horizontalOffset = table.getEntry("tx");
    return horizontalOffset.getDouble(ENTRY_NOT_FOUND);
  }
}

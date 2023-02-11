package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Navx extends SubsystemBase {
  private AHRS ahrs = new AHRS();

  public Navx() {
    calibrate();

    ShuffleboardTab navxTab = Shuffleboard.getTab("navx");
    navxTab.addDouble("Current Pitch", this::getPitch);
    navxTab.addDouble("Current Roll", this::getRoll);
    navxTab.addDouble("Current Yaw", this::getYaw);
    navxTab.addBoolean("Is Calibrating", ahrs::isCalibrating);
  }

  public void calibrate() {
    ahrs.calibrate();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public double getYaw() {
    return ahrs.getYaw();
  }
}

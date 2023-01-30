package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    private CANSparkMax motor;

    ShuffleboardTab grabberTab = Shuffleboard.getTab("Grabber");
    public Grabber() {
      final boolean MOTOR_INVERSION = true;
      final int MOTOR_ID = 6;

      motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
      motor.setInverted(MOTOR_INVERSION);

      grabberTab.addBoolean("Is Closed", this::isFullyClosed);
      grabberTab.addBoolean("Is Open", this::isFullyOpen);
      grabberTab.add(this);
    }

    private final double SPEED = 0.2;
    public CommandBase fullyOpenGrabber() {
        return open(SPEED).until(this::isFullyOpen);
    }

    public CommandBase fullyCloseGrabber() {
        return close(SPEED).until(this::isFullyClosed);
    }

    private final int OPEN_LIMIT_SWITCH_ID = 1;
    private final DigitalInput openLimitSwitch = new DigitalInput(OPEN_LIMIT_SWITCH_ID);
    private boolean isFullyOpen() {
        return !openLimitSwitch.get();
    }

    private final int CLOSED_LIMIT_SWITCH_ID = 0;
    private final DigitalInput closedLimitSwitch = new DigitalInput(CLOSED_LIMIT_SWITCH_ID);
    private boolean isFullyClosed() {
        return !closedLimitSwitch.get();
    }

    private CommandBase open(double speed) {
        CommandBase openCommand = startEnd(() -> motor.set(speed), () -> motor.set(0));
        openCommand.addRequirements(this);
        return openCommand;
        
    }

    private CommandBase close(double speed) {
        return open(-speed);
    }
    
}

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }
}

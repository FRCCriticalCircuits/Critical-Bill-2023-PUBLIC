// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team9062.robot.Subsystems.ArmSubsystem;

public class Robot extends TimedRobot {
  //private DriveSubsystem driveSubsystem;
  private ArmSubsystem armSubsystem;
  //private CriticalLED led;
  private RobotContainer robotContainer;

  public Robot() {
    super(kDefaultPeriod);

    // = DriveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    armSubsystem.setArmBrake(false);

    /* 
    led.scheduleLEDCommand(
      new strobeColor(
        led,
        400,
        Color.kRed
      )
    );
    */
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    armSubsystem.setArmBrake(false);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    armSubsystem.setArmBrake(true);
  }

  @Override
  public void autonomousInit() {
    armSubsystem.setArmBrake(true);

    if(robotContainer.getAutonomousCommand() != null) {
      robotContainer.getAutonomousCommand().schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    armSubsystem.setArmBrake(true);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
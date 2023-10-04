// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team9062.robot.Commands.teleopCommand;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.CriticalLED.CriticalLED;
import frc.team9062.robot.Utils.CriticalLED.PresetCommands.staticColor;
import frc.team9062.robot.Utils.CriticalLED.PresetCommands.strobeColor;

public class Robot extends TimedRobot {
  private DriveSubsystem driveSubsystem;
  private ArmSubsystem armSubsystem;
  private teleopCommand teleopCommand;
  private CriticalLED led = new CriticalLED(
    Constants.IDs.LED_PORT, 
    Constants.LED_BUFFER_LENGTH
  );

  public Robot() {
    super(kDefaultPeriod);

    driveSubsystem = DriveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    teleopCommand = new teleopCommand();
  }

  @Override
  public void robotInit() {
    driveSubsystem.setDefaultCommand(
      teleopCommand
    );

    armSubsystem.setDefaultCommand(
      teleopCommand
    );

    led.startLEDManagerThread();
    led.scheduleLEDCommand(
      new strobeColor(
        led,
        400,
        Color.kRed
      )
    );
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().schedule(teleopCommand);
    led.scheduleLEDCommand(
      new staticColor(
        led, 
        Color.kGreen
      )
    );
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Commands.teleopCommand;
import frc.team9062.robot.Subsystems.DriveSubsystem;


public class RobotContainer {
  private DriveSubsystem system = new DriveSubsystem();
  public RobotContainer() {
    configureBindings();

    system.setDefaultCommand(
      new teleopCommand(
        system,
        true
      )
    );
  }
  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

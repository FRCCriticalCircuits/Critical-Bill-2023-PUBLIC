// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Commands.teleopArm;
import frc.team9062.robot.Commands.teleopDrive;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.IO;

public class RobotContainer {
  private DriveSubsystem driveSubsystem;
  private ArmSubsystem armSubsystem;
  private IO io;
  private SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem = DriveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    io = IO.getInstance();
    
    autoChooser.addOption("TEST TAXI", "Test Taxi");
    autoChooser.addOption("MIDDLE TAXI", "Middle Taxi");
    autoChooser.addOption("CLEARSIDE TAXI", "Clearside Taxi");
    autoChooser.addOption("BUMPSIDE TAXI", "BumpSide Taxi");

    SmartDashboard.putData("AUTO SELECTOR", autoChooser);

    driveSubsystem.setDefaultCommand(
      new teleopDrive()
    );

    armSubsystem.setDefaultCommand(
      new teleopArm()
    );

    configureBindings();
  }
  
  private void configureBindings() {}

  public Command getAutonomousCommand(){
    return null; //new testAuto().testAutoCommand();
  }
}
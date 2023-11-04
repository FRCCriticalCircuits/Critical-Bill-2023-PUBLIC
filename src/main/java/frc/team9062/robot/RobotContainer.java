// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team9062.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team9062.robot.Autos.bumpsideTaxi;
import frc.team9062.robot.Autos.clearsideTaxi;
import frc.team9062.robot.Autos.middleTaxi;
import frc.team9062.robot.Autos.testAuto;
import frc.team9062.robot.Commands.LimedarCommands;
import frc.team9062.robot.Commands.teleopArm;
import frc.team9062.robot.Commands.teleopDrive;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Subsystems.LimedarSubsystem;

public class RobotContainer {
  private DriveSubsystem driveSubsystem;
  private ArmSubsystem armSubsystem;
  private LimedarSubsystem limedar_sys = new LimedarSubsystem();
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem = DriveSubsystem.getInstance();
    armSubsystem = ArmSubsystem.getInstance();
    
    autoChooser.addOption("TEST TAXI", new testAuto().testAutoCommand());
    autoChooser.addOption("MIDDLE TAXI", new middleTaxi().middleTaxiCommand());
    autoChooser.addOption("CLEARSIDE TAXI", new clearsideTaxi().clearsideTaxiCommand());
    autoChooser.addOption("BUMPSIDE TAXI", new bumpsideTaxi().bumpsideTaxiCommand());

    SmartDashboard.putData("AUTO SELECTOR", autoChooser);

    driveSubsystem.setDefaultCommand(
      new teleopDrive()
    );

    armSubsystem.setDefaultCommand(
      new teleopArm()
    );
    
    limedar_sys.setDefaultCommand(
      new LimedarCommands(limedar_sys)
    );

    configureBindings();
  }
  
  private void configureBindings() {}

  public Command getAutonomousCommand(){
    return autoChooser.getSelected();
  }
}
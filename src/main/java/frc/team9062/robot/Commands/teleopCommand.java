package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.IO;
import frc.team9062.robot.Utils.SwerveDriveController;

public class teleopCommand extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private SwerveDriveController driveController;
    private IO io;

    public teleopCommand(){
      driveSubsystem = DriveSubsystem.getInstance();
      driveController = new SwerveDriveController();
      armSubsystem = ArmSubsystem.getInstance();
      io = IO.getInstance();

      addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
      driveController.drive(
        io.getDriverLeftY() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS,
        io.getDriverLeftX() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
        io.getDriverRightX() * Constants.PhysicalConstants.MAX_ANGULAR_SPEED_METERS, 
        true
      );

      if(Math.abs(io.getOperatorLeftY()) > 0) {
        armSubsystem.setArm(io.getOperatorLeftY() * 0.5);
      }
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

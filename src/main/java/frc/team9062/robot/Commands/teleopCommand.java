package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.ControllerBinds;
import frc.team9062.robot.Utils.IO;
import frc.team9062.robot.Utils.SwerveDriveController;

public class teleopCommand extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private SwerveDriveController driveController;
    private ControllerBinds binds;
    private IO io;
    private double lastVel = 0;
    private boolean manualArmControl = true;

    public teleopCommand(){
      driveSubsystem = DriveSubsystem.getInstance();
      driveController = new SwerveDriveController();
      armSubsystem = ArmSubsystem.getInstance();
      io = IO.getInstance();
      binds = ControllerBinds.getInstance();

      addRequirements(driveSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
      // Drive
      driveController.drive(
        io.getDriverLeftY() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS,
        io.getDriverLeftX() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
        io.getDriverRightX() * Constants.PhysicalConstants.MAX_ANGULAR_SPEED_METERS, 
        true
      );

      if(manualArmControl) {
        armSubsystem.setArm(-io.getOperatorLeftY());
      } else {
        
      }

      armSubsystem.setShoulder(io.getOperatorRightY());

      // Intake Binds
      if(binds.ConeIntake()) {
        armSubsystem.setConeIntake();
      } else if(binds.CubeIntake()) {
        armSubsystem.setCubeIntake();
      } else if(binds.Outtake()) {
        armSubsystem.setIntake(-0.5);
      } else {
        armSubsystem.setIntake(0);
      }

      // Use rate of change in robot velocity to set controller feedback
      double velocity = Math.sqrt(Math.pow(driveSubsystem.getXAccelVel(), 2) + Math.pow(driveSubsystem.getYAccelVel(), 2));
      double accel = (velocity - lastVel) / Constants.LOOP_TIME_S;
      
      accel = Math.abs(accel) > Constants.PhysicalConstants.MAX_WHEEL_SPEED_METERS ? 1 : accel / Constants.PhysicalConstants.MAX_WHEEL_SPEED_METERS;
      io.setDriverRumble((accel * Math.signum(accel)) * Constants.Controller.DRIVER_RUMBLE_STRENGTH);

      lastVel = velocity;
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
package frc.team9062.robot.Commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Subsystems.ArmSubsystem.ARM_STATE;
import frc.team9062.robot.Subsystems.ArmSubsystem.INTAKE_STATE;
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
    private boolean clip0, clip180, clip90, clippingToggled = false, manualArmControl = true;
    private Debouncer toggleDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    private Debouncer clippingDebouncer = new Debouncer(0.2, DebounceType.kBoth);

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
      if(!clippingToggled) {
        driveController.drive(
          io.getDriverLeftY() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS,
          io.getDriverLeftX() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
          io.getDriverRightX() * Constants.PhysicalConstants.MAX_ANGULAR_SPEED_METERS, 
          true
        );
      } else {
        double heading = clip0 ? 180 : clip180 ? 0 : clip90 ? 90 : -90;

        driveController.driveWithHeading(
          io.getDriverLeftY(), 
          io.getDriverLeftX(), 
          heading
        );
      }

      if(binds.resetHeading()) driveSubsystem.resetHeading();

      if(clippingDebouncer.calculate(Math.abs(io.getDriverRightX()) > 0)) clippingToggled = false;
      if(toggleDebouncer.calculate(Math.abs(io.getOperatorLeftY()) > 0)) manualArmControl = true;
      
      // Arm Controls
      if(manualArmControl) {
        armSubsystem.setArm(-io.getOperatorLeftY());
        armSubsystem.setCurrentArmState(ARM_STATE.MANUAL);
      }

      if(binds.setArmStowed()) {
        armSubsystem.setCurrentArmState(ARM_STATE.HOLD);
        manualArmControl = false;
      } else if(binds.setArmHigh()) {
        armSubsystem.setCurrentArmState(ARM_STATE.HIGH);
        manualArmControl = false;
      } else if(binds.setArmMid()) {
        armSubsystem.setCurrentArmState(ARM_STATE.MID);
        manualArmControl = false;
      } else if(binds.setArmDoubleSub()) {
        armSubsystem.setCurrentArmState(ARM_STATE.DOUBLE_SUB);
        manualArmControl = false;
      }

      // 
      if(io.getDriverPOVUP()) {
        setClipsFalse();
        clip0 = true;
        clippingToggled = true;
      } else if(io.getDriverPOVRIGHT()) {
        setClipsFalse();
        clip90 = true;
        clippingToggled = true;
      } else if(io.getDriverPOVDOWN()) {
        setClipsFalse();
        clip180 = true;
        clippingToggled = true;
      } else if(io.getDriverPOVLEFT()) {
        setClipsFalse();
        clippingToggled = true;
      }

      armSubsystem.setShoulder(io.getOperatorRightY());

      // Intake Binds
      if(binds.ConeIntake()) {
        armSubsystem.setConeIntake();
      } else if(binds.CubeIntake()) {
        armSubsystem.setCubeIntake();
      } else if(binds.Outtake()) {
        armSubsystem.setIntake(-0.5);
        if(!armSubsystem.objectDetected()) armSubsystem.setCurrentIntakeState(INTAKE_STATE.IDLE);
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
  
    public void setClipsFalse() {
      clip0 = false;
      clip180 = false;
      clip90 = false;
    } 

    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
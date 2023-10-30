package frc.team9062.robot.Commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.ControllerBinds;
import frc.team9062.robot.Utils.IO;
import frc.team9062.robot.Utils.SwerveDriveController;

public class teleopDrive extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private SwerveDriveController driveController;
    private IO io;
    private ControllerBinds binds;
    private boolean snappingToggled = false;
    private SNAP_ANGLE currentAngle = SNAP_ANGLE.SNAP_0;
    private Debouncer toggleDebouncer = new Debouncer(0.1);
 
    private enum SNAP_ANGLE {
        SNAP_0,
        SNAP_90,
        SNAP_180,
        SNAP_270
    }

    public teleopDrive() {
        driveSubsystem = DriveSubsystem.getInstance();
        driveController = new SwerveDriveController();
        io = IO.getInstance();
        binds = ControllerBinds.getInstance();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(!snappingToggled) {
            driveController.drive(
                io.getDriverLeftY() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
                io.getDriverLeftX() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
                io.getDriverRightX() * Constants.PhysicalConstants.MAX_ANGULAR_SPEED_METERS, 
                true
            );

        } else {
            double snappedAngle;

            switch(currentAngle){
                case SNAP_0:
                    snappedAngle = 180;
                    break;
                case SNAP_90:
                    snappedAngle = 90;
                    break;
                case SNAP_180:
                    snappedAngle = 0;
                    break;
                case SNAP_270:
                    snappedAngle = -90;
                    break;
                default:
                    snappedAngle = 0;
            }

            driveController.driveWithHeading(
                -io.getDriverLeftY() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
                io.getDriverLeftX() * Constants.PhysicalConstants.MAX_TRANSLATION_SPEED_METERS, 
                snappedAngle
            );
        }

        if(binds.resetHeading()) driveSubsystem.resetHeading();

        if(io.getDriverPOVUP()) {
            currentAngle = SNAP_ANGLE.SNAP_0;
            snappingToggled = true;
        } else if(io.getDriverPOVRIGHT()) {
            currentAngle = SNAP_ANGLE.SNAP_90;
            snappingToggled = true;
        } else if(io.getDriverPOVDOWN()){
            currentAngle = SNAP_ANGLE.SNAP_180;
            snappingToggled = true;
        } else if(io.getDriverPOVLEFT()) {
            currentAngle = SNAP_ANGLE.SNAP_270;
            snappingToggled = true;
        }

        if(snappingToggled && toggleDebouncer.calculate(Math.abs(io.getDriverRightX()) > 0)) snappingToggled = false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
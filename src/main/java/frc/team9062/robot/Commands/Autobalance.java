package frc.team9062.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Utils.SwerveDriveController;

public class Autobalance extends CommandBase{
    private DriveSubsystem driveSubsystem;
    private SwerveDriveController driveController;
    private PIDController stage1Controller, stage2Controller;
    private double threshold;
    private boolean isReversed, onChargeStation;

    public Autobalance(boolean isReversed, boolean onChargeStation) {
        driveSubsystem = DriveSubsystem.getInstance();
        driveController = new SwerveDriveController();

        this.isReversed = isReversed;
        this.onChargeStation = onChargeStation;

        stage1Controller = new PIDController(
            Constants.TunedConstants.STAGE1_AUTOBALANCE_P, 
            Constants.TunedConstants.STAGE1_AUTOBALANCE_I, 
            Constants.TunedConstants.STAGE1_AUTOBALANCE_D
        );

        stage2Controller = new PIDController(
            Constants.TunedConstants.STAGE2_AUTOBALANCE_P, 
            Constants.TunedConstants.STAGE2_AUTOBALANCE_I, 
            Constants.TunedConstants.STAGE2_AUTOBALANCE_D
        );

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {}
}

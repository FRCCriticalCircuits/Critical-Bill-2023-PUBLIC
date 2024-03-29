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
    private double chargeStationThreshold, threshold = 10;
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

        stage2Controller.setTolerance(0.3);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        /* 
        led.scheduleLEDCommand(
            new strobeColor(
                led, 
                300, 
                Color.kYellow
            )
        );
        */
    }

    @Override
    public void execute() {
        if(!onChargeStation) {
            driveController.drive(
                0, isReversed ? 1 : -1, 0, true
            );

            if(Math.abs(driveSubsystem.getRoll()) > chargeStationThreshold) {
                onChargeStation = true;
            }
        } else {
            if(Math.abs(driveSubsystem.getRoll()) > threshold) {
                driveController.drive(
                    0,
                    isReversed ? 
                        (stage1Controller.calculate(driveSubsystem.getRoll(), 0)) : 
                        -stage1Controller.calculate(driveSubsystem.getRoll(), 0),
                    0,
                    true
                );
            } else {
                driveController.drive(
                    0,
                    isReversed ? 
                        (stage2Controller.calculate(driveSubsystem.getRoll(), 0)) : 
                        -stage2Controller.calculate(driveSubsystem.getRoll(), 0),
                    0, 
                    true
                );

                if(stage2Controller.atSetpoint()) {
                    driveSubsystem.setXWheel();

                /* 
                    led.scheduleLEDCommand(
                        new staticColor(
                            led, 
                            Color.kGreen
                        )
                    );
                */
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
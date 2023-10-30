package frc.team9062.robot.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Subsystems.ArmSubsystem;

public class setShoulder extends CommandBase{
    private ArmSubsystem armSubsystem;
    
    public setShoulder() {
        armSubsystem = ArmSubsystem.getInstance();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        armSubsystem.setShoulder(-1);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setShoulder(0);
    }

    @Override
    public boolean isFinished() {
        if(armSubsystem.shoulderInPosition()) {
            return true;
        } else {
            return false;
        }
    }
}

package frc.team9062.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Subsystems.LimedarSubsystem;

public class LimedarCommands extends CommandBase{
    private LimedarSubsystem limedar_system;

    public LimedarCommands(LimedarSubsystem system){
        this.limedar_system = system;
        addRequirements(limedar_system);
    }

    @Override
    public void initialize() {
      limedar_system.set(0.24);
    }
  
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

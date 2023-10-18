package frc.team9062.robot.Autos;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class TestAuto extends CommandBase {
    DriveSubsystem driveSubsystem;

    public TestAuto() {
        driveSubsystem = DriveSubsystem.getInstance();

        addRequirements(driveSubsystem);
    }

    public Command testAutoCommand() {
        
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.reset(), 
                driveSubsystem
            )
        );
    }
}

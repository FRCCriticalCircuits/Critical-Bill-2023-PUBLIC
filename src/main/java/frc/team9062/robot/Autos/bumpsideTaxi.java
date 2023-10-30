package frc.team9062.robot.Autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class bumpsideTaxi {
    private DriveSubsystem driveSubsystem;
    private AutoConfigs autoConfigs;

    public bumpsideTaxi() {
        driveSubsystem = DriveSubsystem.getInstance();
        autoConfigs = new AutoConfigs();
    }

    public Command bumpsideTaxiCommand() {
        List<PathPlannerTrajectory> bumpsideTaxiPathGroup = PathPlanner.loadPathGroup(
            "Bumpside Taxi", 
            new PathConstraints(1.2, 0.8),
            new PathConstraints(1.2, 0.8)
        );
        
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetPose(bumpsideTaxiPathGroup.get(0).getInitialHolonomicPose()), 
                driveSubsystem
            ),
            autoConfigs.generateAutobuilder(driveSubsystem).fullAuto(bumpsideTaxiPathGroup)
        );
    }
}

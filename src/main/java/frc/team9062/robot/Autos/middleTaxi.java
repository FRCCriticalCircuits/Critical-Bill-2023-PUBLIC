package frc.team9062.robot.Autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class middleTaxi {
    private DriveSubsystem driveSubsystem;
    private AutoConfigs autoConfigs = new AutoConfigs();

    public middleTaxi() {
        driveSubsystem = DriveSubsystem.getInstance();
    }

    public Command middleTaxiCommand() {
        List<PathPlannerTrajectory> middleTaxiPathGroup = PathPlanner.loadPathGroup(
            "Middle Taxi", 
            new PathConstraints(1.1, 0.8),
            new PathConstraints(1.1, 0.8)
        );

        SwerveAutoBuilder autoBuilder = autoConfigs.generateAutobuilder(driveSubsystem);

        return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetPose(middleTaxiPathGroup.get(0).getInitialHolonomicPose()), 
                driveSubsystem
            ),
            autoBuilder.fullAuto(middleTaxiPathGroup)
        );
    }
}

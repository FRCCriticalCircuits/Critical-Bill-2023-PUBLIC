package frc.team9062.robot.Autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team9062.robot.Commands.Autobalance;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class testAuto {
    private DriveSubsystem driveSubsystem;
    private AutoConfigs autoConfigs = new AutoConfigs();

    public testAuto() {
        driveSubsystem = DriveSubsystem.getInstance();
    }  

    public Command testAutoCommand() {
        List<PathPlannerTrajectory> pathgroup = PathPlanner.loadPathGroup(
            "Test Taxi", 
            new PathConstraints(1, 0.4),
            new PathConstraints(1, 0.4)
        );

        return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetPose(pathgroup.get(0).getInitialHolonomicPose()), 
                driveSubsystem
            ),
            //autoConfigs.generateAutobuilder(driveSubsystem).fullAuto(pathgroup)
            new Autobalance(true, true)
        );
    }
}

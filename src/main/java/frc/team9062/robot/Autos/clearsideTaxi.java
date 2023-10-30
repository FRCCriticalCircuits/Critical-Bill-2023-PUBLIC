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

public class clearsideTaxi {
    private DriveSubsystem driveSubsystem;
    private AutoConfigs autoConfigs;

    public clearsideTaxi() {
        driveSubsystem = DriveSubsystem.getInstance();
        autoConfigs = new AutoConfigs();
    }

    public Command clearsideTaxiCommand() {
        List<PathPlannerTrajectory> clearsideTaxiPathGroup = PathPlanner.loadPathGroup(
            "Clearside Taxi", 
            new PathConstraints(1.2, 0.8),
            new PathConstraints(1.2, 0.8)
        );

        SwerveAutoBuilder autoBuilder = autoConfigs.generateAutobuilder(driveSubsystem);

        return new SequentialCommandGroup(
            new InstantCommand(
                () -> driveSubsystem.resetPose(clearsideTaxiPathGroup.get(0).getInitialHolonomicPose()), 
                driveSubsystem
            ),
            autoBuilder.fullAuto(clearsideTaxiPathGroup)
        );
    }
}

package frc.team9062.robot.Autos;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Autos.AutoCommands.setShoulder;
import frc.team9062.robot.Commands.Autobalance;
import frc.team9062.robot.Subsystems.ArmSubsystem;
import frc.team9062.robot.Subsystems.DriveSubsystem;
import frc.team9062.robot.Subsystems.ArmSubsystem.ARM_STATE;

public class AutoConfigs {
    private HashMap<String, Command> eventMap = new HashMap<>();
    private ArmSubsystem armSubsystem;

    public AutoConfigs() {
        armSubsystem = ArmSubsystem.getInstance();

        eventMap.put("setShoulder", new setShoulder());
        eventMap.put("setArmMid", new InstantCommand(() -> armSubsystem.setCurrentArmState(ARM_STATE.LOW), armSubsystem));
        eventMap.put("WaitFor3", new WaitCommand(3.5));
        eventMap.put("setArmHome", new InstantCommand(() -> armSubsystem.setCurrentArmState(ARM_STATE.HOME), armSubsystem));
        eventMap.put("Outake", new SequentialCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> armSubsystem.setIntake(-0.8), armSubsystem),
                new WaitCommand(0.2)
            ),
            new InstantCommand(() -> armSubsystem.setIntake(0.05), armSubsystem)
        ));
        eventMap.put("setArmAndShoulderIn3", 
            new ParallelRaceGroup(
                new ParallelRaceGroup(
                    new setShoulder(),
                    new InstantCommand(() -> armSubsystem.setCurrentArmState(ARM_STATE.HIGH))
                ),
                new WaitCommand(3)
            )
        );
        eventMap.put("balance", new Autobalance(true, true));
    }
    
    public static PIDConstants generateThetaConstants() {
        return new PIDConstants(
            Constants.TunedConstants.AUTO_PID_THETA_P, 
            Constants.TunedConstants.AUTO_PID_THETA_I, 
            Constants.TunedConstants.AUTO_PID_THETA_D
        );
    }

    public static PIDConstants generateTranslationConstants() {
        return new PIDConstants(
            Constants.TunedConstants.AUTO_PID_TRANSLATION_P, 
            Constants.TunedConstants.AUTO_PID_TRANSLATION_I, 
            Constants.TunedConstants.AUTO_PID_TRANSLATION_D
        );
    }
    
    public SwerveAutoBuilder generateAutobuilder(DriveSubsystem driveSubsystem) {
        return new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetPose, 
            generateTranslationConstants(), 
            generateThetaConstants(), 
            driveSubsystem::OutputChassisSpeeds, 
            eventMap,
            true,
            driveSubsystem
        );
    }

    public SwerveAutoBuilder generateAutobuilderWithoutEvents(DriveSubsystem driveSubsystem) {
        return new SwerveAutoBuilder(
            driveSubsystem::getPose,
            driveSubsystem::resetPose, 
            generateTranslationConstants(), 
            generateThetaConstants(), 
            driveSubsystem::OutputChassisSpeeds, 
            null,
            true,
            driveSubsystem
        );
    }
}

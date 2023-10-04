package frc.team9062.robot.Utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class SwerveDriveController {
    private DriveSubsystem driveSubsystem;

    public SwerveDriveController() {
        driveSubsystem = DriveSubsystem.getInstance();
    }

    public void drive( DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean isfieldRelative ) {
        ChassisSpeeds chassisSpeeds = isfieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x.getAsDouble(), y.getAsDouble(), theta.getAsDouble(), driveSubsystem.getRotation2d()
            ) : 
            new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble());

        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.KINEMATIS.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.OutputModuleInfo(swerveModuleStates);
    }

    public void drive( double x, double y, double theta, boolean isfieldRelative ) {
        ChassisSpeeds chassisSpeeds = isfieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, theta, driveSubsystem.getRotation2d()
            ) : 
            new ChassisSpeeds(x, y, theta);

        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.KINEMATIS.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.OutputModuleInfo(swerveModuleStates);
    }

    public void adjustDynamics() {}

    public void convertToSwerveModuleStates() {}
}

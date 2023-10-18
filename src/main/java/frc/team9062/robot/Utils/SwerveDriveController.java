package frc.team9062.robot.Utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class SwerveDriveController {
    private DriveSubsystem driveSubsystem;
    private ProfiledPIDController thetaController;

    public SwerveDriveController() {
        driveSubsystem = DriveSubsystem.getInstance();

        thetaController = new ProfiledPIDController(
            Constants.TunedConstants.THETA_PID_P, 
            Constants.TunedConstants.THETA_PID_I, 
            Constants.TunedConstants.THETA_PID_D, 
            new Constraints(
                Constants.TunedConstants.THETA_MAX_VEL, 
                Constants.TunedConstants.THETA_MAX_ACCEL
            )
        );
        thetaController.enableContinuousInput(-180, 180);
    }

    public void drive( DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, boolean isfieldRelative ) {
        ChassisSpeeds chassisSpeeds = isfieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x.getAsDouble(), y.getAsDouble(), theta.getAsDouble(), driveSubsystem.getRotation2d()
            ) : 
            new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble());

        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.OutputModuleInfo(swerveModuleStates);
    }

    public void drive( double x, double y, double theta, boolean isfieldRelative ) {
        ChassisSpeeds chassisSpeeds = isfieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x, y, theta, driveSubsystem.getRotation2d()
            ) : 
            new ChassisSpeeds(x, y, theta);

        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.OutputModuleInfo(swerveModuleStates);
    }

    public void driveWithHeading(double x, double y, double targetHeading) {
        double theta = thetaController.calculate(driveSubsystem.getHeading(), targetHeading);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, theta, driveSubsystem.getRotation2d()
        );

        SwerveModuleState[] swerveModuleStates = Constants.PhysicalConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        
        driveSubsystem.OutputModuleInfo(swerveModuleStates);
    }

    public void adjustDynamics(ChassisSpeeds desiredSpeeds) {
        
    }
}

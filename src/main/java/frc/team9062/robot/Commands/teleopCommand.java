package frc.team9062.robot.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team9062.robot.Constants.PhysicalConstants;
import frc.team9062.robot.Subsystems.DriveSubsystem;

public class teleopCommand extends CommandBase{
    private final DriveSubsystem drive_subsystem;
    private double x,y,turn;
    private double gain = 1;
    private boolean field_oriented;

    public teleopCommand(DriveSubsystem drive_subsystem , boolean field_oriented){
        this.drive_subsystem = drive_subsystem;
        this.field_oriented = field_oriented;
        addRequirements(drive_subsystem);
    }

    public teleopCommand(DriveSubsystem drive_subsystem ,boolean field_oriented , double gain){
        this.drive_subsystem = drive_subsystem;
        this.gain = gain;
        this.field_oriented = field_oriented;
        addRequirements(drive_subsystem);
    }

    /**
     * 
     * @param input: read from controller
     * @return 0-> if input < deadzone(0.1)
     */
    public double deadzone_calc(double input){
        if(Math.abs(input) < 0.1){
            return 0;
        }
        else{
            return input;
        }
    }

    @Override
    public void initialize() {
        drive_subsystem.setBrakeMode(true);
    }
  
    @Override
    public void execute() {
        x = drive_subsystem.driverJoystick.getLeftY() * gain;
        y = drive_subsystem.driverJoystick.getLeftX() * gain;
        turn = -drive_subsystem.driverJoystick.getRightX() * gain;
        
        x = deadzone_calc(x);
        y = deadzone_calc(y);
        turn = deadzone_calc(turn);

        final double x_speed = x * PhysicalConstants.MAX_METERS_PER_SECOND;
        final double y_speed = y * PhysicalConstants.MAX_METERS_PER_SECOND;
        final double turn_speed = turn * PhysicalConstants.MAX_METERS_PER_SECOND;

        SwerveModuleState[] moduleStates = PhysicalConstants.KINEMATIS.toSwerveModuleStates(
            !field_oriented 
            ? new ChassisSpeeds(x_speed, y_speed, -turn_speed) : 
            ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, -turn_speed, drive_subsystem.getRotation2d()) 
            /*
                ChassisSpeeds chassis_speeds;
                if (!field_oriented) {
                chassis_speeds = new ChassisSpeeds(x_speed, y_speed, turn_speed);
                } else {
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, -turn_speed, drive_subsystem.getRotation2d());
                }
            */
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, PhysicalConstants.MAX_METERS_PER_SECOND);
        drive_subsystem.setStates(moduleStates);
    }
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

package frc.team9062.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants.IDs;
import frc.team9062.robot.Constants.TunedConstants;
import frc.team9062.robot.Constants.PhysicalConstants;

public class DriveSubsystem extends SubsystemBase{
    public SwerveModule frontLeft = new SwerveModule(
      IDs.FRONT_LEFT_FORWARD_ID,
      IDs.FRONT_LEFT_ROTATION_ID,
      IDs.FRONT_LEFT_CANCODER_ID,
      56.6 - 114,
      false,
      false,
      false
    );

    public SwerveModule frontRight = new SwerveModule(
        IDs.FRONT_RIGHT_FORWARD_ID, 
        IDs.FRONT_RIGHT_ROTATION_ID,
        IDs.FRONT_RIGHT_CANCODER_ID,
        -64.863 + 129,
        false,
        false,
        false
    );

    public SwerveModule rearLeft = new SwerveModule(
        IDs.REAR_LEFT_FORWARD_ID, 
        IDs.REAR_LEFT_ROTATION_ID,
        IDs.REAR_LEFT_CANCODER_ID,
        -65 + 130,
        false,
        false,
        false
    );

    public SwerveModule rearRight = new SwerveModule(
        IDs.REAR_RIGHT_FORWARD_ID, 
        IDs.REAR_RIGHT_ROTATION_ID,
        IDs.REAR_RIGHT_CANCODER_ID,
        -50 - 104,
        false,
        false,
        false
    );

    public XboxController driverJoystick = new XboxController(0);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private double GYRO_offset = 0;


  public DriveSubsystem() {
    frontLeft.setForwardPID(
        TunedConstants.PIDF0_DRIVE_P, 
        TunedConstants.PIDF0_DRIVE_I, 
        TunedConstants.PIDF0_DRIVE_D, 
        TunedConstants.PIDF0_DRIVE_F, 0
    );

    frontLeft.setTurnPID(
        TunedConstants.PIDF0_TURN_P, 
        TunedConstants.PIDF0_TURN_I, 
        TunedConstants.PIDF0_TURN_D, 
        TunedConstants.PIDF0_TURN_F, 0
    );

    frontRight.setForwardPID(
        TunedConstants.PIDF0_DRIVE_P, 
        TunedConstants.PIDF0_DRIVE_I, 
        TunedConstants.PIDF0_DRIVE_D, 
        TunedConstants.PIDF0_DRIVE_F, 0
    );

    frontRight.setTurnPID(
        TunedConstants.PIDF0_TURN_P, 
        TunedConstants.PIDF0_TURN_I, 
        TunedConstants.PIDF0_TURN_D, 
        TunedConstants.PIDF0_TURN_F, 0
    );

    rearLeft.setForwardPID(
        TunedConstants.PIDF0_DRIVE_P, 
        TunedConstants.PIDF0_DRIVE_I, 
        TunedConstants.PIDF0_DRIVE_D, 
        TunedConstants.PIDF0_DRIVE_F, 0
    );

    rearLeft.setTurnPID(
        TunedConstants.PIDF0_TURN_P, 
        TunedConstants.PIDF0_TURN_I, 
        TunedConstants.PIDF0_TURN_D, 
        TunedConstants.PIDF0_TURN_F, 0
    );

    rearRight.setForwardPID(
        TunedConstants.PIDF0_DRIVE_P, 
        TunedConstants.PIDF0_DRIVE_I, 
        TunedConstants.PIDF0_DRIVE_D, 
        TunedConstants.PIDF0_DRIVE_F, 0
    );

    rearRight.setTurnPID(
        TunedConstants.PIDF0_TURN_P, 
        TunedConstants.PIDF0_TURN_I, 
        TunedConstants.PIDF0_TURN_D, 
        TunedConstants.PIDF0_TURN_F, 0
    );

    new Thread(() -> {
        try {
          Thread.sleep(2000);
          resetHeading();
        } catch (Exception e) {
        }
      }
    ).start();
  }

  public void setBrakeMode(boolean mode) {
    frontLeft.setBrakeMode(mode);
    frontRight.setBrakeMode(mode);
    rearLeft.setBrakeMode(mode);
    rearRight.setBrakeMode(mode);
  }

  /**
   * Resets the robot's encoders, gyro and odometry
   */
   public void reset(){
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    rearLeft.resetEncoder();
    rearRight.resetEncoder();
    resetHeading();
  }

  public void resetHeading() {
    gyro.zeroYaw();
  }

  /**
   * RETURNS THE HEADING OF THE ROBOT
   * IN ROTATION2D
   *
   * @return heading
  */
  public double getHeading() {
    return gyro.getYaw() * -1 - GYRO_offset;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontleft = frontLeft.getModulePosition();
    SwerveModulePosition frontright = frontRight.getModulePosition();
    SwerveModulePosition rearleft = rearLeft.getModulePosition();
    SwerveModulePosition rearright = rearRight.getModulePosition();

    SwerveModulePosition swerveModulePosition[] = {frontleft, frontright, rearleft, rearright};

    return swerveModulePosition;
  }
  /**
   * Outputs robot information,
   * to be used for autonomous purposes
   * 
   * @param states Desires states of the swerve modules
   * 
   * @return Makes the robot move
   * @return Output Information aboutb the robot on shuffleboard
   */
  public void OutputModuleInfo(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalConstants.MAX_METERS_PER_SECOND);

    frontLeft.setState(states[0], false);
    frontRight.setState(states[1], false);
    rearLeft.setState(states[2], false);
    rearRight.setState(states[3], false);
  }

  public void setStates(SwerveModuleState[] state) {
    frontLeft.setState(state[0], true); // 2
    frontRight.setState(state[1], true); // 3
    rearLeft.setState(state[2], true); // 0
    rearRight.setState(state[3], true); // 1
  }

  public void OutputChassisSpeeds(ChassisSpeeds speeds) {
    OutputModuleInfo(PhysicalConstants.KINEMATIS.toSwerveModuleStates(speeds));
  }

  @Override
  public void periodic() {
  }

}

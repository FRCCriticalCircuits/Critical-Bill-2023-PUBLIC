package frc.team9062.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;
import frc.team9062.robot.Constants.IDs;
import frc.team9062.robot.Constants.PhysicalConstants;

public class DriveSubsystem extends SubsystemBase{
    private static DriveSubsystem instance;

    public enum DRIVE_STATE {
      DRIVING,
      ALIGNING,
      BALANCING,
      LOCK
    }

    public static DriveSubsystem getInstance() {
      if(instance == null) {
        instance = new DriveSubsystem();
      } 
  
      return instance;
    }

    public SwerveModule frontLeft = new SwerveModule(
      IDs.FRONT_LEFT_FORWARD_ID,
      IDs.FRONT_LEFT_ROTATION_ID,
      IDs.FRONT_LEFT_CANCODER_ID,
      157.1,
      false,
      false
    );

    public SwerveModule frontRight = new SwerveModule(
        IDs.FRONT_RIGHT_FORWARD_ID, 
        IDs.FRONT_RIGHT_ROTATION_ID,
        IDs.FRONT_RIGHT_CANCODER_ID,
        63,
        false,
        true
    );

    public SwerveModule rearLeft = new SwerveModule(
        IDs.REAR_LEFT_FORWARD_ID, 
        IDs.REAR_LEFT_ROTATION_ID,
        IDs.REAR_LEFT_CANCODER_ID,
        -66.2,
        false,
        false
    );

    public SwerveModule rearRight = new SwerveModule(
        IDs.REAR_RIGHT_FORWARD_ID, 
        IDs.REAR_RIGHT_ROTATION_ID,
        IDs.REAR_RIGHT_CANCODER_ID,
        121.9,
        false,
        true
    );

  private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    Constants.PhysicalConstants.KINEMATIS, 
    getRotation2d(), 
    getSwerveModulePositions()
  );

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    Constants.PhysicalConstants.KINEMATIS, 
    getRotation2d(), 
    getSwerveModulePositions(), 
    new Pose2d(0, 0, new Rotation2d(0))
  );

  private Field2d field = new Field2d();

  public DriveSubsystem() {
    new Thread(() -> {
        try {
          Thread.sleep(2000);
          resetHeading();
          Thread.currentThread().interrupt();
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
  }

  public void resetHeading() {
    gyro.zeroYaw();
  }

  public double getHeading() {
    return gyro.getYaw() * (Constants.PhysicalConstants.isGyroReversed ? -1 : 1) - Constants.PhysicalConstants.GYRO_OFFSET;
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

  public double getAngleRate() {
    return gyro.getRate();
  }

  public double getXAccelVel() {
    return gyro.getVelocityX();
  }

  public double getYAccelVel() {
    return gyro.getVelocityY();
  }

  public void updateOdometry() {
    poseEstimator.update(getRotation2d(), getSwerveModulePositions());
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
      getRotation2d(), 
      getSwerveModulePositions(), 
      pose
    );
    
    poseEstimator.resetPosition(
      getRotation2d(), 
      getSwerveModulePositions(), 
      pose
    );
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontleft = frontLeft.getModulePosition();
    SwerveModulePosition frontright = frontRight.getModulePosition();
    SwerveModulePosition rearleft = rearLeft.getModulePosition();
    SwerveModulePosition rearright = rearRight.getModulePosition();

    SwerveModulePosition swerveModulePositions[] = {frontleft, frontright, rearleft, rearright};

    return swerveModulePositions;
  }

  public void OutputModuleInfo(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalConstants.MAX_WHEEL_SPEED_METERS);

    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);
  }

  public void OutputChassisSpeeds(ChassisSpeeds speeds) {
    OutputModuleInfo(PhysicalConstants.KINEMATIS.toSwerveModuleStates(speeds));
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getSwerveModulePositions());
    updateOdometry();

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("FIELD", field);

    SmartDashboard.putNumber("FRONT LEFT ANGLE", Math.toDegrees(frontLeft.getAngle()));
    SmartDashboard.putNumber("FRONT RIGHT ANGLE", Math.toDegrees(frontRight.getAngle()));
    SmartDashboard.putNumber("REAR LEFT ANGLE", Math.toDegrees(rearLeft.getAngle()));
    SmartDashboard.putNumber("REAR RIGHT ANGLE", Math.toDegrees(rearRight.getAngle()));

    SmartDashboard.putNumber("ANGLE", getHeading());
  }
}
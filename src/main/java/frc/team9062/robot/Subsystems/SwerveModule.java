package frc.team9062.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax drive;
    private CANSparkMax turn;
    private WPI_CANCoder canCoder;
    private SparkMaxPIDController drivePID, turnPID;
    private RelativeEncoder driveEncoder, turnEncoder;
    //private boolean field_oriented;
    //private PIDController controller = new PIDController(0.003455, 0.000009, 0);

    public SwerveModule(int driveMotor_ID, int turn_ID, int cancoder_ID, double angleOffset, boolean EncoderReversed, boolean driveMotorReversed) {
        drive = new CANSparkMax(driveMotor_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new WPI_CANCoder(cancoder_ID);

        drive.restoreFactoryDefaults();
        turn.restoreFactoryDefaults();

        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();

        driveEncoder = drive.getEncoder();
        turnEncoder = turn.getEncoder();
        
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        canCoder.configMagnetOffset(angleOffset);

        drive.setInverted(driveMotorReversed);
        turn.setInverted(true);

        //drivePID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);
        drive.setSmartCurrentLimit(40);
        turn.setSmartCurrentLimit(40);

        driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4));
        driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4) / 60);
        turnEncoder.setPositionConversionFactor((1 / Constants.PhysicalConstants.TURN_GEAR_RATIO) * Math.PI * 2);
        turnEncoder.setVelocityConversionFactor(((1 / Constants.PhysicalConstants.TURN_GEAR_RATIO) * Math.PI * 2) / 60);
    
        canCoder.configSensorDirection(EncoderReversed);
        canCoder.setPositionToAbsolute();

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(Math.toRadians(canCoder.getAbsolutePosition()));

        drivePID.setP(Constants.TunedConstants.PIDF0_DRIVE_P, 0);
        drivePID.setI(Constants.TunedConstants.PIDF0_DRIVE_I, 0);
        drivePID.setD(Constants.TunedConstants.PIDF0_DRIVE_D, 0);
        drivePID.setFF(Constants.TunedConstants.PIDF0_DRIVE_F, 0);

        turnPID.setP(Constants.TunedConstants.PIDF0_TURN_P, 0);
        turnPID.setI(Constants.TunedConstants.PIDF0_TURN_I, 0);
        turnPID.setD(Constants.TunedConstants.PIDF0_TURN_D, 0);
        turnPID.setFF(Constants.TunedConstants.PIDF0_TURN_F, 0);
        //turnPID.setOutputRange(-Math.PI, Math.PI, 0);
        //turnPID.setIZone(1);

        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        turn.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        drive.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);

        setBrakeMode(true);

        drive.burnFlash();
        turn.burnFlash();
    }

    public void setdrivePID(double p, double i, double d, double f, int slotID) {
        drivePID.setP(p, slotID);
        drivePID.setI(i, slotID);
        drivePID.setD(d, slotID);
        drivePID.setFF(f, slotID);

        drive.burnFlash();
    }    
    
    public void setTurnPID(double p, double i, double d, double f, int slotID) {
        turnPID.setP(p, slotID);
        turnPID.setI(i, slotID);
        turnPID.setD(d, slotID);
        turnPID.setFF(f, slotID);
        //turnPID.setIZone(1);
        //turnPID.setOutputRange(-180, 180);

        turn.burnFlash();
    }

    public void setBrakeMode(Boolean mode){
        if(mode){
            drive.setIdleMode(IdleMode.kBrake);
            turn.setIdleMode(IdleMode.kBrake);
        }else{
            drive.setIdleMode(IdleMode.kCoast);
            turn.setIdleMode(IdleMode.kCoast);        
        }
    }

    public void resetEncoder(){
        driveEncoder.setPosition(0);
        canCoder.setPosition(getAbsoluteAngle());
    }

    public void checkEncoder() {
        if(getAngle() > Math.PI) {
            turnEncoder.setPosition(getAngle() - (2 * Math.PI));
        } else if(getAngle() < -Math.PI) {
            turnEncoder.setPosition(getAngle() + (2 * Math.PI));
        }
    }

    public double closestAngle(double angle) {
        double dir = (angle % (2 * Math.PI)) - (getAngle() % (2 * Math.PI));

        if(Math.abs(dir) > Math.PI){
            dir = -(Math.signum(dir) * (2 * Math.PI)) + dir;
        }

        return dir;
    }

    public void setAngle(double angle) {
        double setpointAngle = closestAngle(angle);
        double setpointAngleInvert = closestAngle(angle + Math.PI);

        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleInvert)){
            turnPID.setReference(getAngle() + setpointAngle, ControlType.kPosition);
        }else{
            turnPID.setReference(getAngle() + setpointAngleInvert, ControlType.kPosition);
        }
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getRotation());
        double velocity = desiredState.speedMetersPerSecond;
        double moduleangle = desiredState.angle.getRadians();
        //drivePID.setReference(velocity, ControlType.kVelocity);
        //drivePID.setReference(
        //    (velocity / Constants.PhysicalConstants.MAX_WHEEL_SPEED_METERS), 
        //    ControlType.kDutyCycle
        //);
        drive.set(velocity / Constants.PhysicalConstants.MAX_WHEEL_SPEED_METERS);
        //turnPID.setReference(moduleangle, ControlType.kPosition);
        setAngle(moduleangle);

        //SmartDashboard.getNumber("TARGET ANGLE " + canCoder.getDeviceID() / 3, moduleangle);
        //SmartDashboard.getNumber("ANGLE " + canCoder.getDeviceID() / 3, moduleangle);
    }

    public double getVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getAngle() {
        return turnEncoder.getPosition();
    }

    public double getAngleCancoder() {
        return canCoder.getPosition();
    }

    public double getAbsoluteAngle() {
        return canCoder.getAbsolutePosition();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(getAngle());
    }

    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
           getVelocity(), 
           getRotation()
        );
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition( 
            getDistance(), 
            getRotation()
        );
    }
    
    @Override
    public void periodic(){
        checkEncoder();
        
    }
}
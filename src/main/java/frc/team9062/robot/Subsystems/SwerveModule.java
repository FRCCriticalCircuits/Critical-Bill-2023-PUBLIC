package frc.team9062.robot.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax forward;
    private CANSparkMax turn;
    private WPI_CANCoder canCoder;
    private SparkMaxPIDController forwardPID, turnPID;
    private RelativeEncoder forwardEncoder, turnEncoder;
    private int driveMotor_ID, turn_ID, cancoder_ID;
    private double gain,angleOffset;
    private boolean EncoderReversed;
    private boolean driveMotorReversed;
    private boolean field_oriented;
    private PIDController controller = new PIDController(0.003455, 0.000009, 0);

    public SwerveModule(int driveMotor_ID, int turn_ID, int cancoder_ID, double angleOffset, boolean EncoderReversed, boolean field_oriented,boolean driveMotorReversed) {
        this.driveMotor_ID = driveMotor_ID;
        this.turn_ID = turn_ID;
        this.cancoder_ID = cancoder_ID;
        this.angleOffset = angleOffset;
        this.EncoderReversed = EncoderReversed;
        this.field_oriented = field_oriented;
        this.driveMotorReversed = driveMotorReversed;

        forward = new CANSparkMax(driveMotor_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new WPI_CANCoder(cancoder_ID);

        forwardPID = forward.getPIDController();
        turnPID = turn.getPIDController();

        forwardEncoder = forward.getEncoder();
        turnEncoder = turn.getEncoder();
        
        forwardPID.setFeedbackDevice(forwardEncoder);
        turnPID.setFeedbackDevice(turnEncoder);
        
        forwardEncoder.setPosition(0.0);
        turnEncoder.setPosition(canCoder.getAbsolutePosition());

        forward.setInverted(driveMotorReversed);
        turn.setInverted(true);
        forwardPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);
        forward.setSmartCurrentLimit(40);
        turn.setSmartCurrentLimit(40);
        canCoder.configSensorDirection(EncoderReversed);
        canCoder.configMagnetOffset(angleOffset);
        canCoder.setPosition(getAbsoluteAngle());
        turnEncoder.setPositionConversionFactor(((1 / Constants.PhysicalConstants.ROTATION_GEAR_RATIO) * 360));
        forwardEncoder.setPositionConversionFactor(Units.inchesToMeters((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4)));
        forwardEncoder.setVelocityConversionFactor(Units.inchesToMeters(((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4))) / 60);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        setBrakeMode(true);

        forward.burnFlash();
        turn.burnFlash();
    }

    public void setForwardPID(double p, double i, double d, double f, int slotID) {
        forwardPID.setP(p, slotID);
        forwardPID.setI(i, slotID);
        forwardPID.setD(d, slotID);
        forwardPID.setFF(f, slotID);

        forward.burnFlash();
    }    
    
    public void setTurnPID(double p, double i, double d, double f, int slotID) {
        turnPID.setP(p, slotID);
        turnPID.setI(i, slotID);
        turnPID.setD(d, slotID);
        turnPID.setFF(f, slotID);
        turnPID.setIZone(1);
        //turnPID.setOutputRange(-180, 180);

        turn.burnFlash();
    }

    public void setBrakeMode(Boolean mode){
        if(mode){
            forward.setIdleMode(IdleMode.kBrake);
            turn.setIdleMode(IdleMode.kBrake);
        }else{
            forward.setIdleMode(IdleMode.kCoast);
            turn.setIdleMode(IdleMode.kCoast);        
        }
    }

    public void resetEncoder(){
        forwardEncoder.setPosition(0);
        canCoder.setPosition(getAbsoluteAngle());
    }

    public double closestAngle(double angle) {
        double dir = (angle % 360) - (getAngleCancoder() % 360);

        if(Math.abs(dir) > 180){
            dir = -(Math.signum(dir) * 360) + dir;
        }
        return dir;
    }

    public void setAngle(double angle) {
        double setpointAngle = closestAngle(angle);
        double setpointAngleInvert = closestAngle(angle + 180);

        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleInvert)){
            gain = 1;
            if(field_oriented){
                turnPID.setReference(getAngleCancoder() + setpointAngle, ControlType.kPosition);
            }else{
                turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + setpointAngle), ControlType.kDutyCycle);
            }
        }else{
            gain = -1;
            if(field_oriented){
                turnPID.setReference(getAngleCancoder() + setpointAngleInvert, ControlType.kPosition);
            }else{
                turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + setpointAngleInvert), ControlType.kDutyCycle);
            }
        }
        
    }
    public void setState(SwerveModuleState state, boolean isOpenLoop){
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getRotation());
        double velocity = desiredState.speedMetersPerSecond;
        double moduleangle = desiredState.angle.getDegrees();
        forwardPID.setReference(velocity, ControlType.kVelocity);
        turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + closestAngle(moduleangle)), ControlType.kDutyCycle);

        SmartDashboard.getNumber("TARGET ANGLE " + cancoder_ID / 3, moduleangle);
        SmartDashboard.getNumber("ANGLE " + cancoder_ID / 3, moduleangle);
    }

    public double getSpeed(){
        return forwardEncoder.getVelocity();
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
        return Rotation2d.fromDegrees(getAngleCancoder());
    }

    public double getDistance() {
        return forwardEncoder.getPosition();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
           getSpeed(), getRotation()
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
    }
}

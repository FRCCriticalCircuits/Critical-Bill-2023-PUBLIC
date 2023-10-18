package frc.team9062.robot.Subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team9062.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private static ArmSubsystem instance;
    private CANSparkMax arm, arm_follower;
    private SparkMaxPIDController armPID;
    private RelativeEncoder armEncoder;
    private ArmFeedforward feedforward;
    private VictorSPX intake;
    private ColorSensorV3 colorSensor;
    private TalonSRX shoulder;
    private BangBangController cubeIntakeController;
    //private boolean hasObject = false;
    private ARM_STATE currentArmState;
    
    public static ArmSubsystem getInstance() {
        if(instance == null) {
            instance = new ArmSubsystem();
        }

        return instance;
    }

    public enum ARM_STATE {
        HIGH,
        MID,
        DOUBLE_SUB,
        HOLD,
        STARTING,
        MANUAL
    }

    public enum SHOULDER_POSITIONS {
        STARTING,
        UPRIGHT,
        FORWARD
    }

    public enum INTAKE_STATES {
        INTAKING,
        HOLDING,
        IDLE
    }

    private HashMap<ARM_STATE, Double> armMap = new HashMap<>();

    public ArmSubsystem() {
        cubeIntakeController = new BangBangController();

        arm = new CANSparkMax(Constants.IDs.ARM, MotorType.kBrushless);
        arm_follower = new CANSparkMax(Constants.IDs.ARM_FOLLOWER, MotorType.kBrushless);
        intake = new VictorSPX(Constants.IDs.ARM_INTAKE);
        colorSensor = new ColorSensorV3(Constants.IDs.INTAKE_COLOR_SENSOR);
        shoulder = new TalonSRX(Constants.IDs.SHOULDER);

        armPID = arm.getPIDController();
        armEncoder = arm.getEncoder();

        arm.restoreFactoryDefaults();
        arm_follower.restoreFactoryDefaults();
        shoulder.configFactoryDefault(1000);

        arm.setSmartCurrentLimit(Constants.PhysicalConstants.ARM_CURRENT_LIMIT);
        arm_follower.setSmartCurrentLimit(Constants.PhysicalConstants.ARM_CURRENT_LIMIT);

        //arm.setClosedLoopRampRate(0.1);
        //arm_follower.setClosedLoopRampRate(0.1);

        shoulder.configVoltageCompSaturation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        intake.configVoltageCompSaturation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        arm.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        arm_follower.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);

        intake.enableVoltageCompensation(true);
        shoulder.enableVoltageCompensation(true);

        intake.configNeutralDeadband(0.04);

        armPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_ARM_VELOCITY, 0);
        armPID.setSmartMotionMaxAccel(Constants.PhysicalConstants.MAX_ARM_ACCELERATION, 0);

        armEncoder.setPositionConversionFactor(1 / Constants.PhysicalConstants.ARM_GEAR_RATIO * Math.PI * 2);
        armEncoder.setVelocityConversionFactor((1 / Constants.PhysicalConstants.ARM_GEAR_RATIO * Math.PI * 2) / 60);
        armEncoder.setPosition(Constants.PhysicalConstants.ARM_STARTING);

        armPID.setP(Constants.TunedConstants.PIDF0_ARM_P, 0);
        armPID.setI(Constants.TunedConstants.PIDF0_ARM_I, 0);
        armPID.setD(Constants.TunedConstants.PIDF0_ARM_D, 0);
        armPID.setFF(Constants.TunedConstants.PIDF0_ARM_F, 0);

        feedforward = new ArmFeedforward(
            Constants.TunedConstants.FEED_ARM_KS, 
            Constants.TunedConstants.FEED_ARM_KG, 
            Constants.TunedConstants.FEED_ARM_KV,
            Constants.TunedConstants.FEED_ARM_KA
        );
    
        arm.setIdleMode(IdleMode.kBrake);
        arm_follower.setIdleMode(IdleMode.kBrake);
        shoulder.setNeutralMode(NeutralMode.Brake);

        arm_follower.follow(arm, false);

        armPID.setFeedbackDevice(armEncoder);

        arm.burnFlash();
        arm_follower.burnFlash();

        // Map arm states to their positions
        armMap.put(ARM_STATE.STARTING, Constants.PhysicalConstants.ARM_STARTING);
        armMap.put(ARM_STATE.HIGH, Constants.PhysicalConstants.ARM_HIGH);
        armMap.put(ARM_STATE.MID, Constants.PhysicalConstants.ARM_MID);
        armMap.put(ARM_STATE.DOUBLE_SUB, Constants.PhysicalConstants.ARM_DOUBLE_SUB);
        armMap.put(ARM_STATE.HOLD, Constants.PhysicalConstants.ARM_STOWED);
    }

    public void setArm(double percentOutput) {
        arm.set(percentOutput);
    }

    public void setIntake(double percentOutput) {
        intake.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    public void setConeIntake() {
        double percentOutput = getSensorProximity() >= Constants.PhysicalConstants.INTAKE_PROXIMITY_THRESHOLD ? 0 : 0.2;
        intake.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    public void setCubeIntake() {
        double percentOutput = cubeIntakeController.calculate(getSensorProximity(), Constants.PhysicalConstants.INTAKE_PROXIMITY_THRESHOLD) == 1 ? 0.2 : 0;
        intake.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    public void setShoulder(double percentOutput) {
        shoulder.set(TalonSRXControlMode.PercentOutput, percentOutput);
    }

    public void setArmPositionSmartMotion(double angleRad) {
        armPID.setReference(
            angleRad, 
            ControlType.kSmartMotion, 
            0
            //feedforward.calculate(angleRad, 0), 
            //ArbFFUnits.kVoltage
        );
    }

    public void setArmPosition(double angleRad) {
        armPID.setReference(
            angleRad, 
            ControlType.kPosition,
            0
        );
    }

    public void setShoulderPosition(double setpoint) {
        shoulder.set(
            TalonSRXControlMode.Position, 
            setpoint
        );
    }

    public void setCurrentArmState(ARM_STATE state) {
        currentArmState = state;
    }

    public void handleArmStates() {
        if(currentArmState != null && currentArmState != ARM_STATE.MANUAL) {
            setArmPositionSmartMotion(armMap.get(currentArmState));
        }
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public double getArmPositionAngle() {
        return Math.toDegrees(armEncoder.getPosition());
    }

    public double getArmVelocity() {
        return armEncoder.getVelocity();
    }
    
    public double getArmVelocityDegrees() {
        return Math.toRadians(armEncoder.getVelocity());
    }

    public double getShoulderPosition() {
        return shoulder.getSelectedSensorPosition();
    }

    public boolean hasObject() {
        return getSensorProximity() > 400;
    }

    public Color getSensorColor() {
        return colorSensor.getColor();
    }

    public int getSensorProximity() {
        return colorSensor.getProximity();
    }

    public double getSensorBlue() {
        return colorSensor.getColor().blue;
    }

    public double getSensorGreen() {
        return colorSensor.getColor().green;
    }

    public double getSensorRed() {
        return colorSensor.getColor().red;
    }

    public double[] getSensorRGB() {
        double[] rgb = {getSensorRed(), getSensorGreen(), getSensorBlue()};

        return rgb;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Detected Color", getSensorRGB());
        SmartDashboard.putNumber("ARM ANGLE", armEncoder.getPosition());
        SmartDashboard.putNumber("SENSOR PROXITY", getSensorProximity());
        SmartDashboard.putNumber("SHOULDER POSITION", getShoulderPosition());
        
        handleArmStates();
    }
 }

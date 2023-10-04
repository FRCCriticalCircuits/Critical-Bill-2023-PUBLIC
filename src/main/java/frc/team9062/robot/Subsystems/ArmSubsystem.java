package frc.team9062.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
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
import edu.wpi.first.wpilibj.I2C.Port;
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

    public static ArmSubsystem getInstance() {
        if(instance == null) {
            instance = new ArmSubsystem();
        }

        return instance;
    }

    public ArmSubsystem() {
        arm = new CANSparkMax(Constants.IDs.ARM, MotorType.kBrushless);
        arm_follower = new CANSparkMax(Constants.IDs.ARM_FOLLOWER, MotorType.kBrushless);
        intake = new VictorSPX(Constants.IDs.ARM_INTAKE);
        colorSensor = new ColorSensorV3(Port.kMXP);

        armPID = arm.getPIDController();
        armEncoder = arm.getEncoder();

        arm.restoreFactoryDefaults();
        arm_follower.restoreFactoryDefaults();

        arm.setSmartCurrentLimit(Constants.PhysicalConstants.ARM_CURRENT_LIMIT);
        arm_follower.setSmartCurrentLimit(Constants.PhysicalConstants.ARM_CURRENT_LIMIT);

        intake.configVoltageCompSaturation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        arm.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);
        arm_follower.enableVoltageCompensation(Constants.PhysicalConstants.NOMINAL_VOLTAGE);

        armPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_ARM_VELOCITY, 0);
        //armPID.setSmartMotionMaxAccel(Constants.PhysicalConstants.MAX_ARM_ACCELERATION, 0);

        armEncoder.setPositionConversionFactor(1 / Constants.PhysicalConstants.ARM_GEAR_RATIO * Math.PI * 2);
        armEncoder.setVelocityConversionFactor((1 / Constants.PhysicalConstants.ARM_GEAR_RATIO * Math.PI * 2) / 60);

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

        arm_follower.follow(arm, false);

        armPID.setFeedbackDevice(armEncoder);

        arm.burnFlash();
        arm_follower.burnFlash();
    }

    public void setArm(double percentOutput) {
        arm.set(percentOutput);
    }

    public void setIntake(double percentOutput) {
        intake.set(VictorSPXControlMode.PercentOutput, percentOutput);
    }

    public void setArmPositionSmartMotion(double angleRad) {
        armPID.setReference(
            angleRad, 
            ControlType.kSmartMotion, 
            0, 
            feedforward.calculate(angleRad, 2), 
            ArbFFUnits.kVoltage
        );
    }

    public void setArmPosition(double angleRad) {
        armPID.setReference(
            angleRad, 
            ControlType.kPosition
        );
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

    public Color getSensorColor() {
        return colorSensor.getColor();
    }

    public double[] getSensorRGB() {
        double[] rgb = {getSensorRed(), getSensorGreen(), getSensorBlue()};

        return rgb;
    }

    public int getSensorProximity() {
        return colorSensor.getProximity();
    }

    public int getSensorBlue() {
        return colorSensor.getBlue();
    }

    public int getSensorGreen() {
        return colorSensor.getGreen();
    }

    public int getSensorRed() {
        return colorSensor.getRed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumberArray("Detected Color", getSensorRGB());
    }
 }

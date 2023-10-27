package frc.team9062.robot.Utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.team9062.robot.Constants;

public class IO {
    private static IO instance;
    private XboxController driverGamepad;
    private XboxController operatorGamepad;
    private CriticalDeadband deadband;

    public IO() {
        driverGamepad = new XboxController(Constants.Controller.DRIVER_ID);
        operatorGamepad = new XboxController(Constants.Controller.OPERATOR_ID);
        deadband = new CriticalDeadband(0.15);
        deadband.setMinMax(-1, 1);
    }

    public static IO getInstance() {
        if(instance == null) {
            instance = new IO();
        }

        return instance;
    }

    public double getDriverLeftX() {
        return deadband.applydeadband(() -> driverGamepad.getLeftX());
    }

    public double getDriverLeftY() {
        return deadband.applydeadband(() -> driverGamepad.getLeftY());
    }

    public double getDriverRightX() {
        return deadband.applydeadband(() -> driverGamepad.getRightX());
    }

    public double getDriverRightY() {
        return deadband.applydeadband(() -> driverGamepad.getRightY());
    }

    public boolean getDriverLeftTrigger() {
        return driverGamepad.getLeftTriggerAxis() > 0.8;
    }

    public boolean getDriverRightTrigger() {
        return driverGamepad.getRightTriggerAxis() > 0.8;
    }

    public boolean getDriverLeftBumper() {
        return driverGamepad.getLeftBumper();
    }

    public boolean getDriverRightBumper() {
        return driverGamepad.getRightBumper();
    }

    public boolean getDriverLeftStickButton() {
        return driverGamepad.getLeftStickButton();
    }

    public boolean getDriverRightStickButton() {
        return driverGamepad.getRightStickButton();
    }

    public boolean getDriverXButton() {
        return driverGamepad.getXButton();
    }

    public boolean getDriverYButton() {
        return driverGamepad.getYButton();
    }

    public boolean getDriverAButton() {
        return driverGamepad.getAButton();
    }

    public boolean getDriverBButton() {
        return driverGamepad.getBButton();
    }

    public boolean getDriverPOVUP() {
        return driverGamepad.getPOV() == 0;
    }

    public boolean getDriverPOVRIGHT() {
        return driverGamepad.getPOV() == 90;
    }

    public boolean getDriverPOVDOWN() {
        return driverGamepad.getPOV() == 180;
    }

    public boolean getDriverPOVLEFT() {
        return driverGamepad.getPOV() == 270;
    }

    public double getOperatorLeftX() {
        return deadband.applydeadband(() -> operatorGamepad.getLeftX());
    }

    public double getOperatorLeftY() {
        return deadband.applydeadband(() -> operatorGamepad.getLeftY());
    }

    public double getOperatorRightX() {
        return deadband.applydeadband(() -> operatorGamepad.getRightX());
    }

    public double getOperatorRightY() {
        return deadband.applydeadband(() -> operatorGamepad.getRightY());
    }

    public boolean getOperatorLeftTrigger() {
        return operatorGamepad.getLeftTriggerAxis() > 0.8;
    }

    public boolean getOperatorRightTrigger() {
        return operatorGamepad.getRightTriggerAxis() > 0.8;
    }

    public boolean getOperatorLeftBumper() {
        return operatorGamepad.getLeftBumper();
    }

    public boolean getOperatorRightBumper() {
        return operatorGamepad.getRightBumper();
    }

    public boolean getOperatorLeftStickButton() {
        return operatorGamepad.getLeftStickButton();
    }

    public boolean getOperatorRightStickButton() {
        return operatorGamepad.getRightStickButton();
    }

    public boolean getOperatorXButton() {
        return operatorGamepad.getXButton();
    }

    public boolean getOperatorYButton() {
        return operatorGamepad.getYButton();
    }

    public boolean getOperatorAButton() {
        return operatorGamepad.getAButton();
    }

    public boolean getOperatorBButton() {
        return operatorGamepad.getBButton();
    }

    public boolean getOperatorPOVUP() {
        return operatorGamepad.getPOV() == 0;
    }

    public boolean getOperatorPOVRIGHT() {
        return operatorGamepad.getPOV() == 90;
    }

    public boolean getOperatorPOVDOWN() {
        return operatorGamepad.getPOV() == 180;
    }

    public boolean getOperatorPOVLEFT() {
        return operatorGamepad.getPOV() == 270;
    }

    public void setDriverRumble(double value) {
        driverGamepad.setRumble(RumbleType.kBothRumble, value);
    }

    public void setOperatorRumble(double value) {
        driverGamepad.setRumble(RumbleType.kBothRumble, value);
    }
}
package frc.team9062.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.team9062.robot.Constants;

public class IO {
    private static IO instance;
    private XboxController driverGamepad;
    private GenericHID operatorGamepad;
    private CriticalDeadband deadband;

    public IO() {
        driverGamepad = new XboxController(Constants.Controller.DRIVER_ID);
        operatorGamepad = new GenericHID(Constants.Controller.OPERATOR_ID);
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

    public double getOperatorLeftX() {
        return deadband.applydeadband(operatorGamepad.getRawAxis(0));
    }

    public double getOperatorLeftY() {
        return deadband.applydeadband(operatorGamepad.getRawAxis(1));
    }
}

package frc.team9062.robot.Utils;

import edu.wpi.first.math.filter.Debouncer;

public class ControllerBinds {
    private static ControllerBinds instance;
    private static IO io;
    private Debouncer debouncer = new Debouncer(0.5);

    public ControllerBinds() {
        io = IO.getInstance();
    }

    public static ControllerBinds getInstance() {
        if(instance == null) {
            instance = new ControllerBinds();
        }

        return instance;
    }

    public boolean CubeIntake() {
        return io.getOperatorLeftTrigger();
    }

    public boolean ConeIntake() {
        return io.getOperatorRightTrigger();
    }

    public boolean Outtake() {
        return io.getOperatorXButton();
    }

    public boolean setArmStowed() {
        return io.getOperatorAButton();
    }

    public boolean setArmHigh() {
        return io.getOperatorRightBumper() && io.getOperatorYButton();
    }

    public boolean setArmMid() {
        return io.getOperatorRightBumper() && io.getOperatorXButton();
    }

    public boolean setArmLow() {
        return io.getOperatorRightBumper() && io.getOperatorAButton();
    }

    public boolean setArmHighInverted() {
        return io.getOperatorLeftBumper() && io.getOperatorYButton();
    }

    public boolean setArmMidInverted() {
        return io.getOperatorLeftBumper() && io.getOperatorXButton();
    }

    public boolean resetHeading() {
        return debouncer.calculate(io.getDriverYButton());
    }
}

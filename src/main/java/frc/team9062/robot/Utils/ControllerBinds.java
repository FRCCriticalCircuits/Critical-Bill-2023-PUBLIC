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
        return io.getOperatorRightBumper();
    }

    public boolean setArmHigh() {
        return io.getOperatorYButton();
    }

    public boolean setArmMid() {
        return io.getOperatorBButton();
    }

    public boolean setArmLow() {
        return io.getOperatorAButton();
    }

    public boolean setArmDoubleSub() {
        return io.getOperatorLeftBumper();
    }

    public boolean setArmOrigin() {
        return io.getOperatorLeftStickButton();
    }

    public boolean resetHeading() {
        return debouncer.calculate(io.getDriverYButton());
    }
}

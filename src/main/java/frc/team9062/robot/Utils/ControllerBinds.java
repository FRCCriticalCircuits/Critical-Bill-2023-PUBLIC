package frc.team9062.robot.Utils;

public class ControllerBinds {
    private static ControllerBinds instance;
    private static IO io;

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
}

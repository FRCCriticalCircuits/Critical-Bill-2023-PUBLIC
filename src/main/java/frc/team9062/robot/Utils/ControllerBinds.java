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
}

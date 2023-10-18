package frc.team9062.robot.Utils;

public class RobotState {
    private GamePiece activeGamePiece;
    private static RobotState instance;

    public static RobotState getInstance() {
        if(instance == null) {
            instance = new RobotState();
        }

        return instance;
    }
    
    public RobotState() {
        activeGamePiece = GamePiece.NONE;
    }

    public GamePiece getActiveGamePiece() {
        return activeGamePiece;
    }
    
    public void setActiveGamePieve(GamePiece desiredGamePiece) {
        activeGamePiece = desiredGamePiece;
    }
}

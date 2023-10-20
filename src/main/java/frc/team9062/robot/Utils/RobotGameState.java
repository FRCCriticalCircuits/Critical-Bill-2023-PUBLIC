package frc.team9062.robot.Utils;

public class RobotGameState {
    private GamePiece activeGamePiece;
    private static RobotGameState instance;

    public static RobotGameState getInstance() {
        if(instance == null) {
            instance = new RobotGameState();
        }

        return instance;
    }
    
    public RobotGameState() {
        activeGamePiece = GamePiece.NONE;
    }

    public GamePiece getActiveGamePiece() {
        return activeGamePiece;
    }
    
    public void setActiveGamePieve(GamePiece desiredGamePiece) {
        activeGamePiece = desiredGamePiece;
    }
}

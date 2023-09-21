package frc.team9062.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.team9062.robot.Constants;

public class IO {
    private static IO instance;
    private XboxController driverGamepad;
    private GenericHID operatorGamepad;

    public IO() {
        driverGamepad = new XboxController(Constants.Controller.DRIVER_ID);
        operatorGamepad = new GenericHID(Constants.Controller.OPERATOR_ID);
    }

    public static IO getInstance() {
        if(instance == null) {
            instance = new IO();
        }

        return instance;
    }
}

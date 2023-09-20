package frc.robot.Utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

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

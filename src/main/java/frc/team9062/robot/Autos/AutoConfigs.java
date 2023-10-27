package frc.team9062.robot.Autos;

import frc.team9062.robot.Constants;

public class AutoConfigs {
    public static class testAuto {
        public static final double MAX_MODULE_SPEED = 4.8;
        public static final double MAX_TRANSLATION_VELOCITY = 4.3;
        public static final double MAX_TRANSLATION_ACCEL = 0.4;
    }

    /*
    public static PIDConstants generateThetaConstants() {
        return new PIDConstants(
            Constants.TunedConstants.AUTO_PID_THETA_P, 
            Constants.TunedConstants.AUTO_PID_THETA_I, 
            Constants.TunedConstants.AUTO_PID_THETA_D
        );
    }

    public static PIDConstants generateTranslationConstants() {
        return new PIDConstants(
            Constants.TunedConstants.AUTO_PID_TRANSLATION_P, 
            Constants.TunedConstants.AUTO_PID_TRANSLATION_I, 
            Constants.TunedConstants.AUTO_PID_TRANSLATION_D
        );
    }
    */
}

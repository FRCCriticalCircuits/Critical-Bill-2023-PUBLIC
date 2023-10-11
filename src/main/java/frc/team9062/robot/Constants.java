package frc.team9062.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Constants {
    public static class Controller {
        public static int DRIVER_ID = 0;
        public static int OPERATOR_ID = 1;

        public static double GAMEPAD_THRESHOLD = 0.1;

        public static double DRIVER_RUMBLE_STRENGTH = 0.8; // Cannot be larger than 1. The lower the number the less feedback strength the driver would feel.
    }

    public static class IDs{
        // -----------------------------------------------
        // DRIVE ID'S
        // -----------------------------------------------

        public static final int FRONT_LEFT_FORWARD_ID = 1;
        public static final int FRONT_LEFT_ROTATION_ID = 2;
        public static final int FRONT_LEFT_CANCODER_ID = 3;
        public static final double FRONT_LEFT_OFFSET = 56.6 - 114;
    
        public static final int FRONT_RIGHT_FORWARD_ID = 4;
        public static final int FRONT_RIGHT_ROTATION_ID = 5;
        public static final int FRONT_RIGHT_CANCODER_ID = 6;
        public static final double FRONT_RIGHT_OFFSET = -64.893 + 129;
    
        public static final int REAR_LEFT_FORWARD_ID = 7;
        public static final int REAR_LEFT_ROTATION_ID = 8;
        public static final int REAR_LEFT_CANCODER_ID = 9;
        public static final double REAR_LEFT_OFFSET = -65 + 130;
    
        public static final int REAR_RIGHT_FORWARD_ID = 10;
        public static final int REAR_RIGHT_ROTATION_ID = 11;
        public static final int REAR_RIGHT_CANCODER_ID = 12;
        public static final double REAR_RIGHT_OFFSET = -50 - 104;
    

        // -----------------------------------------------

        // -----------------------------------------------
        // ARM ID'S
        // -----------------------------------------------

        public static final int ARM = 15;
        public static final int ARM_FOLLOWER = 16;
        
        public static final int ARM_INTAKE = 17;
        public static final Port INTAKE_COLOR_SENSOR = I2C.Port.kOnboard;

        public static final int SHOULDER = 18;

        public static final int SHUFFLER_ARM = 20;
        public static final int SHUFFLER_INTAKE_LEFT = 21;
        public static final int SHUFFLER_INTAKE_RIGHT = 22;

        // ------------------------------------------------

        public static final int LIMELIGHT_SERVO = 0;

        public static final int LED_PORT = 3;
    }

    public static class TunedConstants {
        // ------------------------------------------------
        // DRIVE
        // ------------------------------------------------

        public static final double PIDF0_DRIVE_P = 0; // Previously 0.003405
        public static final double PIDF0_DRIVE_I = 0;
        public static final double PIDF0_DRIVE_D = 0;
        public static final double PIDF0_DRIVE_F = 0; // Previously 0.315

        public static final double FEED_DRIVE_KV = 0;
        public static final double FEED_DRIVE_KS = 0;
        public static final double FEED_DRIVE_KA = 0;

        public static final double PIDF0_TURN_P = 0.292;
        public static final double PIDF0_TURN_I = 0;
        public static final double PIDF0_TURN_D = 0;
        public static final double PIDF0_TURN_F = 0.0008;

        // ------------------------------------------------

        // ------------------------------------------------
        // ARM
        // ------------------------------------------------

        public static final double PIDF0_ARM_P = 0;
        public static final double PIDF0_ARM_I = 0;
        public static final double PIDF0_ARM_D = 0;
        public static final double PIDF0_ARM_F = 0;

        public static final double FEED_ARM_KV = 0;
        public static final double FEED_ARM_KS = 0;
        public static final double FEED_ARM_KA = 0;
        public static final double FEED_ARM_KG = 0;

        public static final double PID0_SHUFFLER_P = 0;
        public static final double PID0_SHUFFLER_I = 0;
        public static final double PID0_SHUFFLER_D = 0;

        public static final double FEED_SHUFFLER_KV = 0;
        public static final double FEED_SHUFFLER_KS = 0;
        public static final double FEED_SHUFFLER_KA = 0;
        public static final double FEED_SHUFFLER_KG = 0;

        // -------------------------------------------------

        // -------------------------------------------------
        // AUTOBALANCE PID
        // -------------------------------------------------
        
        public static final double STAGE1_AUTOBALANCE_P = 0;
        public static final double STAGE1_AUTOBALANCE_I = 0;
        public static final double STAGE1_AUTOBALANCE_D = 0;

        public static final double STAGE2_AUTOBALANCE_P = 0;
        public static final double STAGE2_AUTOBALANCE_I = 0;
        public static final double STAGE2_AUTOBALANCE_D = 0;

        // -------------------------------------------------

        public static final double THETA_PID_P = 0;
        public static final double THETA_PID_I = 0;
        public static final double THETA_PID_D = 0;
        public static final double THETA_MAX_VEL = 1;
        public static final double THETA_MAX_ACCEL = 1;
    }

    public static class PhysicalConstants {
        public static final double TRACK_WIDTH = 24 - 2.625; // Square Base, so no track length needed
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(TRACK_WIDTH);

        public final static SwerveDriveKinematics KINEMATIS = new SwerveDriveKinematics(
            new Translation2d((TRACK_WIDTH_METERS / 2), (TRACK_WIDTH_METERS / 2)),
            new Translation2d((TRACK_WIDTH_METERS / 2), -(TRACK_WIDTH_METERS / 2)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2), (TRACK_WIDTH_METERS / 2)),
            new Translation2d(-(TRACK_WIDTH_METERS / 2), -(TRACK_WIDTH_METERS / 2))
        );

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4;
        public static final double DRIVE_WHEEL_DIAMTER_METERS = Units.inchesToMeters(4);

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double TURN_GEAR_RATIO = 150 / 7;

        public static final double ARM_GEAR_RATIO = 480; // Larger Tooth: 480, Smaller Tooth: 240

        public static final double SHUFFLER_GEAR_RATIO = 1.25;

        public static final double MAX_TRANSLATION_SPEED_METERS = 4.3;
        public static final double MAX_ANGULAR_SPEED_METERS = 3.6;
        public static final double MAX_WHEEL_SPEED_METERS = 4.5;

        public static final double MAX_ARM_VELOCITY = 1.5;
        public static final double MAX_ARM_ACCELERATION = 1.2;

        // -------------------------------------------------
        // CURRENT
        // -------------------------------------------------

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int TURN_CURRENT_LIMIT = 40;

        public static final int ARM_CURRENT_LIMIT = 40;

        public static final int SHUFFLER_ARM_CURRENT_LIMIT = 30;

        public static final int INTAKE_CURRENT_LIMIT = 20;
        public static final int INTAKE_HOLD_CURRENT_LIMIT = 5;

        public static final int SHOULDER_CURRENT_LIMIT = 50;

        // --------------------------------------------------
    
        public static final double NOMINAL_VOLTAGE = 12; // For Comp saturation

        public static final boolean isGyroReversed = true;
        public static final double GYRO_OFFSET = 180;

        // --------------------------------------------------
        // ENCODER OFFSETS
        // --------------------------------------------------
        
        public static final double FRONT_LEFT_OFFSET = 0;
        public static final double FRONT_RIGHT_OFFSET = 0;
        public static final double REAR_LEFT_OFFSET = 0;
        public static final double REAR_RIGHT_OFFSET = 0;

        // --------------------------------------------------

        // --------------------------------------------------
        // ARM POSITIONS
        // --------------------------------------------------
        
        public static double ARM_FRONT_HOLD = -0.6;
        public static double ARM_STARTING = 1.968;
        public static double ARM_CUBE_HIGH = 0;

        // --------------------------------------------------

        public static final int INTAKE_PROXIMITY_THRESHOLD = 400;
        
    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;

    public static final int LED_BUFFER_LENGTH = 57;
}

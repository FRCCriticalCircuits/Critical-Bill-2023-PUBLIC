package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class Constants {
    public static class Controller {
        public static int DRIVER_ID = 0;
        public static int OPERATOR_ID = 1;
    }

    public static class IDs{
        // -----------------------------------------------
        // DRIVE ID'S
        // -----------------------------------------------

        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int FRONT_LEFT_TURN = 2;
        public static final int FRONT_LEFT_CANCODER = 3;

        public static final int FRONT_RIGHT_DRIVE = 4;
        public static final int FRONT_RIGHT_TURN = 5;
        public static final int FRONT_RIGHT_CANCODER = 6;

        public static final int REAR_LEFT_DRIVE = 7;
        public static final int REAR_LEFT_TURN = 8;
        public static final int REAR_LEFT_CANCODER = 9;

        public static final int REAR_RIGHT_DRIVE = 10;
        public static final int REAR_RIGHT_TURN = 11;
        public static final int REAR_RIGHT_CANCODER = 12;

        // -----------------------------------------------

        // -----------------------------------------------
        // ARM ID'S
        // -----------------------------------------------

        public static final int ARM = 15;
        public static final int ARM_FOLLOWER = 16;
        
        public static final int ARM_INTAKE = 17;
        public final Port INTAKE_COLOR_SENSOR = I2C.Port.kMXP;

        public static final int SHOULDER = 18;

        public static final int SHUFFLER_ARM = 20;
        public static final int SHUFFLER_INTAKE_LEFT = 21;
        public static final int SHUFFLER_INTAKE_RIGHT = 22;

        // ------------------------------------------------

        public static final int LIMELIGHT_SERVO = 0;

        public static final int LED_PORT = 1;
    }

    public static class TunedConstants {
        // ------------------------------------------------
        // DRIVE
        // ------------------------------------------------

        public static final double PIDF0_DRIVE_P = 0;
        public static final double PIDF0_DRIVE_I = 0;
        public static final double PIDF0_DRIVE_D = 0;
        public static final double PIDF0_DRIVE_F = 0;

        public static final double FEED_DRIVE_KV = 0;
        public static final double FEED_DRIVE_KS = 0;
        public static final double FEED_DRIVE_KA = 0;

        public static final double PIDF0_TURN_P = 0;
        public static final double PIDF0_TURN_I = 0;
        public static final double PIDF0_TURN_D = 0;
        public static final double PIDF0_TURN_F = 0;

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
    }

    public static class PhysicalConstants {
        public static final double TRACK_WIDTH = 24; // Square Base, so no track length needed
        public static final double TRACK_WIDTH_METERS = 0.6096;

        public final SwerveDriveKinematics KINEMATIS = new SwerveDriveKinematics(
            new Translation2d(TRACK_WIDTH_METERS, TRACK_WIDTH_METERS),
            new Translation2d(TRACK_WIDTH_METERS, -TRACK_WIDTH_METERS),
            new Translation2d(-TRACK_WIDTH_METERS, TRACK_WIDTH_METERS),
            new Translation2d(-TRACK_WIDTH_METERS, -TRACK_WIDTH_METERS)
        );

        public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4;
        public static final double DRIVE_WHEEL_DIAMTER_METERS = Units.inchesToMeters(4);

        public static final double DRIVE_GEAR_RATIO = 1 / 6.12;
        public static final double TURN_TURN_RATIO = 1 / (150 / 7);

        public static final double ARM_GEAR_RATIO = 1; // Todo: Figure out ratio

        public static final double SHUFFLER_GEAR_RATIO = 1 / 1.25;

        public static final double MAX_TRANSLATION_SPEED_METERS = 3;
        public static final double MAX_ANGULAR_SPEED_METERS = 2;

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
    
        public static final double NOMINAL_VOLTAGE = 10; // For Comp saturation

        public static final boolean GYRO_ISREVERSED = false;

        // --------------------------------------------------
        // ENCODER OFFSETS
        // --------------------------------------------------
        
        public static final double FRONT_LEFT_OFFSET = 0;
        public static final double FRONT_RIGHT_OFFSET = 0;
        public static final double REAR_LEFT_OFFSET = 0;
        public static final double REAR_RIGHT_OFFSET = 0;

        // --------------------------------------------------

        
    }

    public static final double LOOP_TIME_S = 0.02;
    public static final int LOOP_TIME_MS = 20;

    public static final int LED_BUFFER_LENGTH = 60;
}

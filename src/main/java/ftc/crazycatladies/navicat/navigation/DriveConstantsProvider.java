package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

public class DriveConstantsProvider {
    public static double TICKS_PER_REV;
    public static double MAX_RPM;
    public static boolean RUN_USING_ENCODER;
    public static PIDCoefficients MOTOR_VELO_PID;
    public static double WHEEL_RADIUS;
    public static double GEAR_RATIO;
    public static double TRACK_WIDTH;
    public static double kV;
    public static double kA;
    public static double kStatic;

    public static DriveConstraints BASE_CONSTRAINTS;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
    }

    public static void init(Class driveConstants) {
        try {
            TICKS_PER_REV = driveConstants.getField("TICKS_PER_REV").getDouble(null);
            MAX_RPM = driveConstants.getField("MAX_RPM").getDouble(null);
            RUN_USING_ENCODER = driveConstants.getField("RUN_USING_ENCODER").getBoolean(null);
            MOTOR_VELO_PID = (PIDCoefficients) driveConstants.getField("MOTOR_VELO_PID").get(null);
            WHEEL_RADIUS = driveConstants.getField("WHEEL_RADIUS").getDouble(null);
            GEAR_RATIO = driveConstants.getField("GEAR_RATIO").getDouble(null);
            TRACK_WIDTH = driveConstants.getField("TRACK_WIDTH").getDouble(null);
            kV = driveConstants.getField("kV").getDouble(null);
            kA = driveConstants.getField("kA").getDouble(null);
            kStatic = driveConstants.getField("kStatic").getDouble(null);
            BASE_CONSTRAINTS = (DriveConstraints) driveConstants.getField("BASE_CONSTRAINTS").get(null);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException("Provided driveConstants does not contain necessary fields", e);
        } catch (IllegalAccessException e) {
            throw new RuntimeException("One of the necessary fields in the provided driveConstants is not accessible", e);
        }
    }
}

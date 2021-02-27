package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelPeriodicImuLocalizer {
    public static double SIDE_FORWARD_OFFSET = 3.15;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.18; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 12.79; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.02; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.01; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private BNO055IMU imu;

    public StandardTrackingWheelLocalizer(Encoder leftEncoder, Encoder rightEncoder, Encoder middleEncoder, Drive drive) {
        super(Arrays.asList(
                new Pose2d(SIDE_FORWARD_OFFSET, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(SIDE_FORWARD_OFFSET, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ), drive);

        this.leftEncoder = leftEncoder; //new Encoder(hardwareMap.get(DcMotorEx.class, "FR"))
        this.rightEncoder = rightEncoder; //new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        this.frontEncoder = middleEncoder; // new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));

        this.imu = imu;

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
//        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}

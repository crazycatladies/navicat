package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DriveConstants {
    final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    public final PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(15.0, 0.0, 0.0);
    public final boolean RUN_USING_ENCODER = true;
    public double WHEEL_RADIUS;
    public double GEAR_RATIO; // output (wheel) speed / input (motor) speed
    public double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public double kA = 0;
    public double kStatic = 0;
    public double TRACK_WIDTH;

    public DriveConstants(double WHEEL_RADIUS, double GEAR_RATIO, double TRACK_WIDTH) {
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.TRACK_WIDTH = TRACK_WIDTH;
    }

    public DriveConstants(double WHEEL_RADIUS, double GEAR_RATIO, double kV, double kA, double kStatic, double TRACK_WIDTH) {
        this.WHEEL_RADIUS = WHEEL_RADIUS;
        this.GEAR_RATIO = GEAR_RATIO;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.TRACK_WIDTH = TRACK_WIDTH;
    }

    public double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO *2*Math.PI *WHEEL_RADIUS /60.0;
}

    public double getMaxRpm() {
        return MOTOR_CONFIG.getMaxRPM() *
                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
    }

    public double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        return (MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0);
    }

    public double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / getTicksPerSec();
    }
}
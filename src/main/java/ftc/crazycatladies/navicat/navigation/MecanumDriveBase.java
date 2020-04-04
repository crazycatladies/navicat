package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import ftc.crazycatladies.nyan.actuators.DcMotorEx;

public class MecanumDriveBase extends MecanumDrive {
    private final DriveConstants driveConstants;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private Localization localization;

    public MecanumDriveBase(DcMotorEx leftFront, DcMotorEx leftRear, DcMotorEx rightRear,
                            DcMotorEx rightFront, Localization localization, DriveConstants driveConstants) {
        super(driveConstants.kV, driveConstants.kA, driveConstants.kStatic, driveConstants.TRACK_WIDTH);
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.localization = localization;
        this.driveConstants = driveConstants;

        if (localization == null)
            setLocalizer(new MecanumLocalizer(this, false));

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
    }

    public void init() {
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx motor : motors) {
            if (driveConstants.RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (driveConstants.RUN_USING_ENCODER && driveConstants.MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, driveConstants.MOTOR_VELO_PID);
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(driveConstants.encoderTicksToInches(leftFront.getCurrentPosition()));
        wheelPositions.add(driveConstants.encoderTicksToInches(leftRear.getCurrentPosition()));
        wheelPositions.add(driveConstants.encoderTicksToInches(-rightRear.getCurrentPosition()));
        wheelPositions.add(driveConstants.encoderTicksToInches(-rightFront.getCurrentPosition()));
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    protected double getRawExternalHeading() {
        return localization.getRadImuHeading();
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, driveConstants.getMotorVelocityF()
            ));
        }
    }
}
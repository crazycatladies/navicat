package ftc.crazycatladies.navicat.navigation;

import java.util.Arrays;
import java.util.List;

import ftc.crazycatladies.nyan.actuators.DcMotorEx;
import ftc.crazycatladies.nyan.subsystem.Subsystem;

import static java.lang.Math.abs;
import static java.lang.Math.max;

public class MecanumDrive extends Subsystem {

    public static class DriveMotorConfig {
        private final String frontLeft;
        private final String frontRight;
        private final String backLeft;
        private final String backRight;
        private final int flHub;
        private final int frHub;
        private final int blHub;
        private final int brHub;

        public DriveMotorConfig(String frontLeft, int flHub, String frontRight, int frHub,
                                String backLeft, int blHub, String backRight, int brHub) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
            this.flHub = flHub;
            this.frHub = frHub;
            this.blHub = blHub;
            this.brHub = brHub;
        }
    }

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<DcMotorEx> allMotors;

    public MecanumDrive(DriveMotorConfig dmc) {
        this.frontLeft = new DcMotorEx(dmc.frontLeft, dmc.flHub, true);
        this.frontRight = new DcMotorEx(dmc.frontRight, dmc.frHub, false);
        this.backLeft = new DcMotorEx(dmc.backLeft, dmc.blHub, true);
        this.backRight = new DcMotorEx(dmc.backRight, dmc.brHub, false);

        allMotors = Arrays.asList(new DcMotorEx[]{this.frontLeft, this.frontRight, this.backLeft, this.backRight});

        for (DcMotorEx motor : allMotors) {
            subsystems.add(motor);
        }

    }

    public void move(double forwardBackward, double turnAmount, double strafeAmount) {
        double fl = forwardBackward + turnAmount + strafeAmount;
        double fr = forwardBackward + -turnAmount + -strafeAmount;
        double bl = forwardBackward + turnAmount + -strafeAmount;
        double br = forwardBackward + -turnAmount + strafeAmount;

        double max = max(max(max(abs(fl), abs(fr)), abs(bl)), abs(br));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}

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
        private final boolean flReverse;
        private final boolean frReverse;
        private final boolean blReverse;
        private final boolean brReverse;

        public DriveMotorConfig(String frontLeft, int flHub, boolean flReverse,
                                String frontRight, int frHub, boolean frReverse,
                                String backLeft, int blHub, boolean blReverse,
                                String backRight, int brHub, boolean brReverse) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
            this.flHub = flHub;
            this.frHub = frHub;
            this.blHub = blHub;
            this.brHub = brHub;
            this.flReverse = flReverse;
            this.frReverse = frReverse;
            this.blReverse = blReverse;
            this.brReverse = brReverse;
        }
    }

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<DcMotorEx> allMotors;

    public MecanumDrive(DriveMotorConfig dmc) {
        super("MecanumDrive");
        this.frontLeft = new DcMotorEx(dmc.frontLeft, dmc.flHub, !dmc.flReverse);
        this.frontRight = new DcMotorEx(dmc.frontRight, dmc.frHub, !dmc.frReverse);
        this.backLeft = new DcMotorEx(dmc.backLeft, dmc.blHub, !dmc.blReverse);
        this.backRight = new DcMotorEx(dmc.backRight, dmc.brHub, !dmc.brReverse);

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

package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;

import java.util.List;

import ftc.crazycatladies.nyan.subsystem.Subsystem;

public class NavRobot extends Subsystem {
    Navigation nav;

    public NavRobot(boolean doLocalization, Class driveConstants,
                    PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                    MecanumDrive.DriveMotorConfig dmc,
                    String imu1, String imu2) {
        super("NavRobot");
        this.nav = new Navigation(doLocalization, driveConstants,
                TRANSLATIONAL_PID, HEADING_PID, dmc, imu1, imu2, null, null, null, null);
        subsystems.add(nav);
    }

    public NavRobot(boolean doLocalization, Class driveConstants,
                    PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                    MecanumDrive.DriveMotorConfig dmc,
                    String imu1, String imu2, List<Pose2d> trackingWheelPoses, Encoder leftEncoder, Encoder rightEncoder, Encoder middleEncoder) {
        super("NavRobot");
        this.nav = new Navigation(doLocalization, driveConstants,
                TRANSLATIONAL_PID, HEADING_PID, dmc, imu1, imu2, trackingWheelPoses, leftEncoder, rightEncoder, middleEncoder);
        subsystems.add(nav);
        if (trackingWheelPoses != null) {
            addSubsystems(leftEncoder.getMotor(), rightEncoder.getMotor(), middleEncoder.getMotor());
        }
    }

    public void go(BaseTrajectoryBuilder t) {
        nav.go(t);
    }
    public BaseTrajectoryBuilder to(double x, double y, double heading) {
        return nav.to(x, y, heading);
    }
    public BaseTrajectoryBuilder slowTo(double x, double y, double heading) {
        return nav.slowTo(x, y, heading);
    }
    public BaseTrajectoryBuilder splineTo(double x, double y, double heading) {
        return nav.splineTo(x, y, heading);
    }
    public BaseTrajectoryBuilder reverseTo(double x, double y, double heading) {
        return nav.reverseTo(x, y, heading);
    }
    public BaseTrajectoryBuilder reverseSplineTo(double x, double y, double heading) {
        return nav.reverseSplineTo(x, y, heading);
    }

    public Navigation getNav() {
        return nav;
    }

    public void move(double forward, double turn, double strafe) {
        nav.move(forward, turn, strafe);
    }
}

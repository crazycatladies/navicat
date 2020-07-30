package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import ftc.crazycatladies.nyan.subsystem.Subsystem;

public class NavRobot extends Subsystem {
    Navigation nav;

    public NavRobot(boolean doLocalization, Class driveConstants,
                    PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                    MecanumDrive.DriveMotorConfig dmc,
                    String imu1, String imu2) {
        this.nav = new Navigation(doLocalization, driveConstants,
                TRANSLATIONAL_PID, HEADING_PID, dmc, imu1, imu2);
        subsystems.add(nav);
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
}

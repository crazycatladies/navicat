package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import ftc.crazycatladies.nyan.auto.AbstractAuto;
import ftc.crazycatladies.schrodinger.state.State;
import ftc.crazycatladies.schrodinger.state.StateFunction;

public abstract class AbstractNavAuto extends AbstractAuto {
    protected NavRobot navRobot;

    protected void go(BaseTrajectoryBuilder t) {
        navRobot.go(t);
    }
    protected BaseTrajectoryBuilder to(double x, double y, double heading) {
        return navRobot.to(x, y, heading);
    }
    protected BaseTrajectoryBuilder slowTo(double x, double y, double heading) {
        return navRobot.slowTo(x, y, heading);
    }
    protected BaseTrajectoryBuilder splineTo(double x, double y, double heading) {
        return navRobot.splineTo(x, y, heading);
    }
    public BaseTrajectoryBuilder reverseTo(double x, double y, double heading) {
        return navRobot.reverseTo(x, y, heading);
    }
    public BaseTrajectoryBuilder reverseSplineTo(double x, double y, double heading) {
        return navRobot.reverseSplineTo(x, y, heading);
    }

    protected StateFunction waitForNav() {
        return (State state, Object context) -> {
            if (navRobot.getNav().isDone())
                state.next();
        };
    }

    @Override
    protected void autoStop() {
        super.autoStop();
        navRobot.move(0, 0, 0);
    }
}

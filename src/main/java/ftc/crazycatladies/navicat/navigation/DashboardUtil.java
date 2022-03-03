package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
//    private static final double ROBOT_RADIUS = 9; // in
//    private static final double ROBOT_WIDTH = 8;
//    private static final double ROBOT_LENGTH = 14;


    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose, double robotWidth, double robotLength) {
//        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
//        canvas.strokeRect(pose.getX() - robotLength / 2, pose.getY() - robotWidth / 2, robotLength, robotWidth);
        Vector2d p1 = pose.vec().plus(new Vector2d(robotLength / 2, -robotWidth / 2).rotated(pose.getHeading()));
        Vector2d p2 = pose.vec().plus(new Vector2d(robotLength / 2, robotWidth / 2).rotated(pose.getHeading()));
        Vector2d p3 = pose.vec().plus(new Vector2d(-robotLength / 2, robotWidth / 2).rotated(pose.getHeading()));
        Vector2d p4 = pose.vec().plus(new Vector2d(-robotLength / 2, -robotWidth / 2).rotated(pose.getHeading()));
        canvas.strokePolygon(new double[]{p1.getX(), p2.getX(), p3.getX(), p4.getX()},
                new double[]{p1.getY(), p2.getY(), p3.getY(), p4.getY()});
        Vector2d v = pose.headingVec().times(robotLength / 2.0);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
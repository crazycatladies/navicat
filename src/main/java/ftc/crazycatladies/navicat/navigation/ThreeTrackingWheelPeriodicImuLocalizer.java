package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static java.util.Collections.emptyList;

public abstract class ThreeTrackingWheelPeriodicImuLocalizer implements Localizer {

    private final ElapsedTime headingTimer = new ElapsedTime(0);
    private final int headingResetIntervalMs = 100;
    Pose2d poseEstimate = new Pose2d();
    Pose2d poseVelocity = new Pose2d();
    List<Double> lastWheelPositions = emptyList();
    double lastHeading = Double.NaN;
    private final DecompositionSolver forwardSolver;
    private final Drive drive;

    public ThreeTrackingWheelPeriodicImuLocalizer(@NotNull List<Pose2d> wheelPoses, Drive drive) {
        this.drive = drive;

        if (wheelPoses.size() != 3)
            throw new IllegalArgumentException("3 wheel positions must be provided");

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i <= 2; i++) {
            final Vector2d orientationVector = wheelPoses.get(i).headingVec();
            final Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(i, 2,
                    positionVector.getX() * orientationVector.getY()
                            - positionVector.getY() * orientationVector.getX());
        }
        System.out.println(inverseMatrix);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular())
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas) {
        final double[] wheelDeltasArray = new double[]{wheelDeltas.get(0), wheelDeltas.get(1), wheelDeltas.get(2)};
        final double[][] array = new double[][]{wheelDeltasArray};
        RealMatrix realMatrix = MatrixUtils.createRealMatrix(array);
        final RealMatrix rawPoseDelta = forwardSolver.solve(realMatrix.transpose());

        return new Pose2d(rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0));
    }

    @Override
    public void update() {
        final List<Double> wheelPositions = getWheelPositions();
        if (!lastWheelPositions.isEmpty()) {
            final List<Double> wheelDeltas = new ArrayList(3);
            for (int i = 0; i < 3; i++)
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            final Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
            if (headingTimer.milliseconds() > headingResetIntervalMs) {
                poseEstimate = new Pose2d(poseEstimate.getX(), poseEstimate.getY(), drive.getExternalHeading());
                headingTimer.reset();
            }
        }

        final List<Double> wheelVelocities = getWheelVelocities();
        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities);
        }

        lastWheelPositions = wheelPositions;
    }

    public abstract List<Double> getWheelPositions();

    public abstract List<Double> getWheelVelocities();

    @Override
    public void setPoseEstimate(Pose2d poseEstimate) {
        this.poseEstimate = poseEstimate;
        lastWheelPositions = emptyList();
        lastHeading = Double.NaN;
        drive.setExternalHeading(poseEstimate.getHeading());
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }
}

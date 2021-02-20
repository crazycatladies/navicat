package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.jetbrains.annotations.NotNull;
import org.json.JSONObject;

import java.util.Map;

import ftc.crazycatladies.nyan.subsystem.Subsystem;
import ftc.crazycatladies.schrodinger.log.DataLogger;
import ftc.crazycatladies.schrodinger.state.StateMachine;

import static ftc.crazycatladies.navicat.navigation.DriveConstantsProvider.BASE_CONSTRAINTS;
import static ftc.crazycatladies.navicat.navigation.DriveConstantsProvider.TRACK_WIDTH;
import static java.lang.Math.toRadians;

public class Navigation extends Subsystem {
    class TurnContext {
        double angle;
        MotionProfile profile;

        public TurnContext(double angle) {
            this.angle = angle;
        }

        @Override
        public String toString() {
            return "TurnContext{" +
                    "angle=" + angle +
                    ", profile=" + profileToString() + " }";
        }

        @NotNull
        private String profileToString() {
            if (profile == null)
                return "null";

            return "{duration=" +
                    profile.duration() +
                    ", start=" +
                    profile.start() +
                    ", end=" +
                    profile.end() +
                    "}";
        }
    }

    private final PIDFController turnController;
    private final StateMachine<Trajectory> followSM;
    private final StateMachine<TurnContext> turnSM;
    private MecanumDrive drive;
    private MecanumDriveBase driveBase;
    private Localization localization;
    private HolonomicPIDVAFollower follower;

    private Pose2d lastError, currentPose;

    private final DriveConstraints constraints;

    public Navigation(boolean doLocalization, Class driveConstants,
                      PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                      MecanumDrive.DriveMotorConfig dmc,
                      String imu1, String imu2) {
        super("Navigation");
        DriveConstantsProvider.init(driveConstants);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
        if (doLocalization) {
            subsystems.add(localization = new Localization(imu1, imu2));
        }

        drive = new MecanumDrive(dmc);
        driveBase = new MecanumDriveBase(drive.frontLeft, drive.backLeft, drive.backRight,
                drive.frontRight, localization);
        subsystems.add(drive);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        followSM = new StateMachine<Trajectory>("FollowTrajectory");
        followSM.once((state, trajectory) -> {
            follower.followTrajectory(trajectory);
        });
        followSM.repeat((state, trajectory) -> {
            if (!follower.isFollowing()) {
                driveBase.setDriveSignal(new DriveSignal());
                state.next();
            } else {
                driveBase.setDriveSignal(follower.update(currentPose));
                lastError = follower.getLastError();
            }
        });

        turnSM = new StateMachine("TurnTo");
        turnSM.once((state, context) -> {
            double heading = currentPose.getHeading();
            context.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(heading, 0, 0, 0),
                    new MotionState(heading + toRadians(context.angle), 0, 0, 0),
                    constraints.maxVel, constraints.maxAccel);
        });
        turnSM.repeat((state, context) -> {
            MotionState targetState = context.profile.get(state.getTimeInState().seconds());
            if (state.getTimeInState().seconds() > context.profile.duration()
                    && Math.abs(currentPose.getHeading() - targetState.getX()) < Math.toRadians(1.0)) {
                driveBase.setDriveSignal(new DriveSignal());
                state.next();
            } else {
                turnController.setTargetPosition(targetState.getX());
                double correction = turnController.update(currentPose.getHeading());
                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                driveBase.setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));
            }
        });
    }

    @Override
    public void loop(Map<Integer, LynxGetBulkInputDataResponse> bulkDataResponse) throws InterruptedException {
        lastError = null;
        // Loop children in order to update wheel positions, then update pose, then run SM
        loopChildren(bulkDataResponse);
        driveBase.updatePoseEstimate();
        currentPose = driveBase.getPoseEstimate();
        runCurrentSM();
    }

    public void move(double forward, double turn, double strafe) {
        drive.move(forward, turn, strafe);
    }

    public TrajectoryBuilder trajectoryBuilder(double maxVel) {
        return trajectoryBuilder(maxVelConstraints(maxVel));
    }

    private MecanumConstraints maxVelConstraints(double maxVel) {
        return new MecanumConstraints(new DriveConstraints(
                maxVel, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk
        ), TRACK_WIDTH);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return trajectoryBuilder(constraints);
    }

    private TrajectoryBuilder trajectoryBuilder(DriveConstraints constraints) {
        return new TrajectoryBuilder(driveBase.getPoseEstimate(), constraints);
    }

    private TrajectoryBuilder reverseTrajectoryBuilder() {
        return new TrajectoryBuilder(driveBase.getPoseEstimate(), true, constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        runSM(followSM, trajectory);
    }

    public void setPoseEstimate(double x, double y, double heading) {
        driveBase.setPoseEstimate(new Pose2d(x, y, toRadians(heading)));
    }

    @Override
    public void log() {
        super.log();

        JSONObject json = DataLogger.createJsonObject(this.getClass().getSimpleName(), null);

        if (lastError != null)
            DataLogger.putOpt(json, "lastError", lastError);

        if (currentPose != null)
            DataLogger.putOpt(json, "currentPose", currentPose);

        if (json.length() > 1) {
            logger.log(json);
        }
    }

    public void go(BaseTrajectoryBuilder t) {
        followTrajectory(t.build());
    }
    public BaseTrajectoryBuilder to(double x, double y, double heading) {
        return trajectoryBuilder().lineToLinearHeading(new Pose2d(x, y, toRadians(heading)), constraints);
    }
    public BaseTrajectoryBuilder slowTo(double x, double y, double heading) {
        return trajectoryBuilder(20).lineToLinearHeading(new Pose2d(x, y, toRadians(heading)), maxVelConstraints(20));
    }
    public BaseTrajectoryBuilder splineTo(double x, double y, double heading) {
        return trajectoryBuilder().splineTo(new Vector2d(x, y), toRadians(heading), constraints);
    }
    public void turnBy(double angle) {
        runSM(turnSM, new TurnContext(angle));
    }
    public BaseTrajectoryBuilder reverseTo(double x, double y, double heading) {
        return reverseTrajectoryBuilder().lineToLinearHeading(new Pose2d(x, y, toRadians(heading)), constraints);
    }
    public BaseTrajectoryBuilder reverseSplineTo(double x, double y, double heading) {
        return reverseTrajectoryBuilder().splineTo(new Vector2d(x, y), toRadians(heading), constraints);
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public MecanumDriveBase getDriveBase() {
        return driveBase;
    }
}

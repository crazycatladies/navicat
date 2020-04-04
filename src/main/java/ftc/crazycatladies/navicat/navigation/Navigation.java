package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;

import org.json.JSONObject;

import java.util.Map;

import ftc.crazycatladies.nyan.subsystem.Subsystem;
import ftc.crazycatladies.schrodinger.log.DataLogger;
import ftc.crazycatladies.schrodinger.state.StateMachine;

public class Navigation extends Subsystem {
    private MecanumDrive drive;
    private MecanumDriveBase driveBase;
    private Localization localization;
    private HolonomicPIDVAFollower follower;

    private Pose2d lastError, currentPose;

    private static DriveConstraints BASE_CONSTRAINTS;
    private final DriveConstraints constraints;
    private DriveConstants driveConstants;

    public Navigation(boolean useImu, DriveConstants driveConstants,
                      PIDCoefficients TRANSLATIONAL_PID, PIDCoefficients HEADING_PID,
                      DriveConstraints baseConstraints, MecanumDrive.DriveMotorConfig dmc,
                      String imu1, String imu2) {
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
        this.BASE_CONSTRAINTS = baseConstraints;
        this.driveConstants = driveConstants;
        if (useImu) {
            subsystems.add(localization = new Localization(imu1, imu2));
        }

        drive = new MecanumDrive(dmc);
        driveBase = new MecanumDriveBase(drive.frontLeft, drive.backLeft, drive.backRight,
                drive.frontRight, localization, driveConstants);
        subsystems.add(drive);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, driveConstants.TRACK_WIDTH);
    }

    @Override
    public void loop(Map<Integer, LynxGetBulkInputDataResponse> bulkDataResponse) {
        lastError = null;
        // Loop children in order to update wheel positions, then update pose, then run SM
        loopChildren(bulkDataResponse);
        driveBase.updatePoseEstimate();
        runCurrentSM();
    }

    public void move(double forward, double turn, double strafe) {
        drive.move(forward, turn, strafe);
    }

    private StateMachine<Trajectory> followTrajectorySM = new StateMachine<Trajectory>("FollowTrajectory")
        .once((state, trajectory) -> {
            follower.followTrajectory(trajectory);
        }).repeat((state, trajectory) -> {
            if (!follower.isFollowing())
                state.transition();
            else {
                currentPose = driveBase.getPoseEstimate();
                driveBase.setDriveSignal(follower.update(currentPose));
                lastError = follower.getLastError();
            }
        }).once((state, trajectory) -> {
            driveBase.setDriveSignal(new DriveSignal());
        });

    public TrajectoryBuilder trajectoryBuilder(double maxVel) {
        return trajectoryBuilder(new MecanumConstraints(new DriveConstraints(
                maxVel, BASE_CONSTRAINTS.maxAccel, BASE_CONSTRAINTS.maxJerk,
                BASE_CONSTRAINTS.maxAngVel, BASE_CONSTRAINTS.maxAngAccel, BASE_CONSTRAINTS.maxAngJerk
        ), driveConstants.TRACK_WIDTH));
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return trajectoryBuilder(constraints);
    }

    private TrajectoryBuilder trajectoryBuilder(DriveConstraints constraints) {
        return new TrajectoryBuilder(driveBase.getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        runSM(followTrajectorySM, trajectory);
    }

    public void setPoseEstimate(Pose2d pose) {
        driveBase.setPoseEstimate(pose);
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
}

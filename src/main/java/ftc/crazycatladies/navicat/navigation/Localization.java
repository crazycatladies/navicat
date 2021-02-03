package ftc.crazycatladies.navicat.navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONObject;

import java.util.Map;

import ftc.crazycatladies.nyan.subsystem.Subsystem;
import ftc.crazycatladies.schrodinger.log.DataLogger;
import ftc.crazycatladies.schrodinger.opmode.OpModeTime;

import static java.lang.Math.toDegrees;

public class Localization extends Subsystem {
    Float radHeading;
    private BNO055IMU imu;
    private String imu1;
    private String imu2;

    public Localization(String imu1, String imu2) {
        super("Localization");
        this.imu1 = imu1;
        this.imu2 = imu2;
    }

    @Override
    public void init(HardwareMap hwMap, OpModeTime time) {
        super.init(hwMap, time);

        BNO055IMU imu1 = hwMap.get(BNO055IMU.class, this.imu1);
        BNO055IMU imu2 = null;

        if (this.imu2 != null)
            imu2 = hwMap.get(BNO055IMU.class, this.imu2);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;

        RobotLog.i("initializing imu ->");
        if (imu1.initialize(parameters)) {
            imu = imu1;
        } else if (imu2 != null) {
            imu2.initialize(parameters);
            imu = imu2;
        }
        RobotLog.i("<- initializing imu");
    }

    @Override
    public void loop(Map<Integer, LynxGetBulkInputDataResponse> bulkDataResponse) throws InterruptedException {
        radHeading = null;

        super.loop(bulkDataResponse);
    }

    @Override
    public void log() {
        super.log();

        if (radHeading != null) {
            JSONObject json = new JSONObject();
            DataLogger.putOpt(json, "type", this.getClass().getSimpleName());
            DataLogger.putOpt(json, "imuHeading", toDegrees(radHeading));
            logger.log(json);
        }
    }

    // Get heading from IMU
    public float getRadImuHeading() {
        if (radHeading == null) {
            radHeading = imu.getAngularOrientation().firstAngle;
        }

        return radHeading;
    }
}

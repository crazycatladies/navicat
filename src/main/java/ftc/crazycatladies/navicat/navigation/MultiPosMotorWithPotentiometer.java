package ftc.crazycatladies.navicat.navigation;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.crazycatladies.nyan.actuators.DcMotorEx;
import ftc.crazycatladies.nyan.sensors.AnalogSensorEx;
import ftc.crazycatladies.schrodinger.opmode.OpModeTime;
import ftc.crazycatladies.schrodinger.state.StateMachine;
import kotlin.jvm.functions.Function2;

/**
 * Moves a motor to set positions defined in the AngularPosition enum
 * @param <P>
 */
public class MultiPosMotorWithPotentiometer<P extends AngularPosition> extends DcMotorEx {

    class MoveContext {
        MotionProfile mp;
        MotionState ms;
        boolean holdPosition;

        public MoveContext(MotionProfile mp, MotionState ms, boolean holdPosition) {
            this.mp = mp;
            this.ms = ms;
            this.holdPosition = holdPosition;
        }

        @Override
        public String toString() {
            if (ms == null)
                return "MoveContext{mp=" + mp +
                        "}, x=" + analogSensor.getAngle() + "}";
            else
                return "MoveContext{mp=" + mp +
                        ", ms={" + ms.getX() + ',' + ms.getV() + ',' + ms.getA() + ',' + ms.getJ() +
                        "}, x=" + analogSensor.getAngle() + "}";
        }
    }

    private final Function2<Double, Double, Double> kF;
    P currentPosition;
    AnalogSensorEx analogSensor;
    StateMachine<MoveContext> moveSM = new StateMachine<>("MultiPosWithPotMoveSM");
    PIDFController pidfController;
    private double maxVel;
    private double maxAccel;
    private double maxJerk;
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kV;
    private final double kA;
    private final double kS;
    /**
     * @param name motor name
     * @param hubNum motor hub
     * @param isForward motor direction
     * @param analogSensor
     * @param kP
     * @param kI
     * @param kD
     * @param maxVel degrees/s
     * @param maxAccel degrees/s/s
     * @param maxJerk degrees/s/s/s
     * @param kV power/degrees/s
     * @param kA power/degrees/s/s
     * @param kS power to overcome static friction
     * @param kF function (position, velocity) -&gt; power
     */
    public MultiPosMotorWithPotentiometer(String name, int hubNum, boolean isForward,
                                          AnalogSensorEx analogSensor, double kP, double kI, double kD,
                                          double maxVel, double maxAccel, double maxJerk,
                                          double kV, double kA, double kS,
                                          Function2<Double, Double, Double> kF) {
        super(name, hubNum, isForward);
        this.analogSensor = analogSensor;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kF = kF;

        moveSM.repeat((state, moveContext) -> {
            if (!moveContext.holdPosition && state.getTimeInState().seconds() > moveContext.mp.duration()) {
//                if (moveContext.holdPosition) {
//                    pidfController.setTargetPosition(moveContext.mp.end().getX());
//                    pidfController.setTargetVelocity(0);
//                    pidfController.setTargetAcceleration(0);
//                    double power = pidfController.update(analogSensor.getAngle());
//                    setPower(-power);
//                } else {
                    //Stop moving & engage brake
                    setPower(0.0);
                    state.next();
//                }
            } else {
                moveContext.ms = moveContext.mp.get(state.getTimeInState().seconds());
                pidfController.setTargetPosition(moveContext.ms.getX());
                pidfController.setTargetVelocity(moveContext.ms.getV());
                pidfController.setTargetAcceleration(moveContext.ms.getA());
                double power = pidfController.update(analogSensor.getAngle());
                setPower(-power);
            }
        });
    }

    @Override
    public void init(HardwareMap hwMap, OpModeTime time) {
        super.init(hwMap, time);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double batteryVoltage = hwMap.voltageSensor.iterator().next().getVoltage();
        pidfController = new PIDFController(new PIDCoefficients(kP, kI, kD),
                kV * batteryVoltage / 12, kA, kS, kF);
        pidfController.setInputBounds(0, 360);
    }

    public void moveTo(P position, double maxVel, double maxAcc, double maxJerk, boolean holdPosition) {
        MotionProfile mp = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(analogSensor.getAngle(), 0, 0),
                new MotionState(position.getPosition(), 0, 0),
                this.maxVel /* deg/s */, maxAccel /* deg/s/s */, this.maxJerk /* deg/s/s/s */);
        runSM(moveSM, new MoveContext(mp, null, holdPosition));
        currentPosition = position;
    }

    public void moveTo(P position, boolean holdPosition) {
        moveTo(position, this.maxVel, this.maxAccel, this.maxJerk, holdPosition);
    }

    public P getCurrentPos() {
        return currentPosition;
    }
}

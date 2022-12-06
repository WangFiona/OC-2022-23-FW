package overcharged.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Encoder;
import overcharged.components.OcGyro2;
import overcharged.components.OcMotor;
import overcharged.components.RotationAxis;
import overcharged.components.TankDrive;
import overcharged.components.TurnType;
import overcharged.linear.components.OcMotorsLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.odometry.Localization;
import overcharged.pid.Controller;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static overcharged.config.RobotConstants.TAG_TD;

/**
 * This class is for Overcharged autonomous
 * Updated by Parthiv Nair on 9/17/2018.
 */
public class TestTurn
        extends TankDrive
{
    //public final OcGyro gyroSensor;
    public final OcGyro2 gyroSensor;

    private OcMotorsLinear motors;

    private LinearOpMode op;

    WaitLinear lp;

    public Localization odometryLocalization;

    //Global switch to disable odometry
    boolean odometryEnabled = false;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.10f;
    private final static float POWER_START = 0.17f;

    Controller           pidRotate, pidDrive;
    //Orientation lastAngles = new Orientation();
    double                  lastAngleL = 0, lastAngleR = 0;
    double                  globalAngleL, globalAngleR;
    double                  power = .30, correction, rotation;
    private final static float LEFT_POWER_ADJ = 0.76f;


    /*
    An obvious question is how did we arrive at .003 for the P value on the turn PID controller. In the case of moving from a non-zero error to zero error, we took the maximum error value, 90, and the power level we want applied at max error, .30 (30%). We divided the power by the max error ( .30 / 90 = .003 ) to determine P.
    Often with just a P value alone, the robot may stall out before completing the turn because the PID controller reduces the power below the level which will move the robot. To fix this, we add some I (integral) value. The I value will compensate for P not reaching the setpoint and start adding power until the robot completes the turn. A good starting I value is P / 100. You can adjust I to get the turn completed in a timely fashion. Note that these values are optimal for a 90 degree turn with 30% power. They will not work as well for other angles or power levels. You would have to compute new values for other angle/power combinations. Here is the above example modified to compute the P and I values for any angle/power combination input to the rotate() function.
    In the case of driving straight, the target and error are the same at the start so the error is zero. So we have to determine how much correction we want to apply for how much error. Experimentation showed that .05 (5%) correction power for 1 degree of error worked well, correcting the error without overshooting too much (wandering). 1 / .05 = .05 for P.
    As always, you can do these calculations to determine a starting P and I values and then adjust then to tune actual robot behavior.
    In most simple cases only a P value is needed. I may be needed if you can't find a P value that reaches the setpoint with out overshooting. A discussion of using the D value is beyond the scope of this lesson.
     */
    public TestTurn(OcMotor driveLeftFront,
                    OcMotor driveLeftBack,
                    OcMotor driveRightFront,
                    OcMotor driveRightBack,
                    Encoder left, Encoder right,
                    OcGyro2 gyro,
                    Localization ol)
    {
        super(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);

        this.gyroSensor = gyro;
        this.odometryLocalization = ol;
        this.motors = new OcMotorsLinear(this.motorsArray);
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
//        pidRotate = new PIDController(.003, .00003, 0);
//        pidRotate = new PIDController(0.08, 0, 0.08);
        pidRotate = new Controller(0.08, 0.0000, 0.04);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        //pidDrive = new PIDController(.04, 0, 0.005);
        // competitoin pidDrive = new PIDController(.03, 0, 0.005);
        pidDrive = new Controller(0.03, 0, 0.005);

        RobotLog.ii(TAG_TD, "initialized");

    }

    public void setLinearOpMode (LinearOpMode op) {
        this.op = op;
        lp = new WaitLinear(op);
        motors.setLinearOpMode(op);
    }

    /**
     * set motor regulation
     * @param mode motor regulation to set to
     */
    public void setMode2(RunMode mode)
            throws InterruptedException
    {
        motors.setMode(mode);
    }

    public void stop2()
            throws InterruptedException
    {
        motors.stop();
    }

    /**
     * set motor power float until it is confirmed.
     * @throws InterruptedException
     */
    public void setZeroPowerBehavior2(ZeroPowerBehavior behavior)
            throws InterruptedException
    {
        motors.setZeroPowerBehavior(behavior);
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition2(int position)
            throws InterruptedException
    {
        motors.setTargetPosition(position);
    }

    public void turn (TurnType turnType,
                      float deg,
                      float maxPower,
                      int timeoutMillis,
                      boolean isAccelerate)
            throws InterruptedException
    {
        // how much to ramp up
        final float Kp_up = turnType == TurnType.TURN_REGULAR ? 0.015f : 0.03f; // 2 per degree
        // how much to ramp down
        final float Kp_down = turnType == TurnType.TURN_REGULAR ? 0.0075f : 0.015f; // 1 per degree

        deg = deg % 360;

        if (timeoutMillis < 0) {
            // 30 milliseconds per degree
            timeoutMillis = (int)Math.abs(deg * 30) + 2000;
        }

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        stop2();

        setMode2(RunMode.RUN_USING_ENCODER);

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        float heading;
        while (op.opModeIsActive() &&
                timeStamp - startTimestamp < timeoutMillis) {

            heading = gyroSensor.getHeading();
            float delta = deg - heading;
            //if (Math.abs(delta) <= 1)
            if (Math.abs(heading) >= Math.abs(deg))
            {
                // done
                break;
            }

            float power;
            if (!isAccelerate)
            {
                power = Math.signum(delta) * (maxPower);
            }
            else {
                if (Math.abs(heading) < Math.abs(deg / 3f)) {
                    // ramp up
                    power = Math.signum(deg) * (Kp_up * Math.abs(heading) + POWER_START);
                    //power = Math.signum(deg) * (maxPower);
                } else {
                    // ramp down
                    power = Kp_down * delta + Math.signum(delta) * POWER_STOP;
                }
            }

            power = Range.clip(power, -maxPower, maxPower);
            setPower(power,
                    power,
                    turnType);

            //op.telemetry.addData("heading", Float.toString(heading));
            //op.telemetry.addData("delta", Float.toString(delta));
            //op.telemetry.addData("power", Float.toString(power));
            //op.telemetry.addData("counter", Integer.toString(++counter));

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        stop2();

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.ee(TAG_TD, "Thread interrupted");
        }
        else if (op.isStopRequested()) {
            RobotLog.ee(TAG_TD, "Stop requested");
        }

        heading = gyroSensor.getHeading();
        RobotLog.ii(TAG_TD, "Turned " + heading + " degree");
        RobotLog.ii(TAG_TD, "Turn took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("heading", heading);

        // the same as swerve drive
        gyroSensor.adjustHeading(deg);
        op.idle();
    }

    public void turn (float deg,

                      float maxPower,
                      int timeoutMillis,
                      boolean isAccelerate)
            throws InterruptedException {
        turn(TurnType.TURN_REGULAR,
                deg,
                maxPower,
                timeoutMillis,
                isAccelerate);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        //lastAngles = gyroSensor.getLAngularOrientation();
        lastAngleL = gyroSensor.getLFirstAngle();
        lastAngleR = gyroSensor.getRFirstAngle();
        globalAngleL = 0;
        globalAngleR = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle(RotationAxis rotationAxis)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Orientation angles = gyroSensor.getAngularOrientation();
        double angleL,angleR;
        //angleL = gyroSensor.getLFirstAngle();
        //angleR = gyroSensor.getRFirstAngle();
        angleL = gyroSensor.getLHeading();
        angleR = gyroSensor.getRHeading();

        double deltaAngleL = angleL - lastAngleL;

        if (deltaAngleL < -180)
            deltaAngleL += 360;
        else if (deltaAngleL > 180)
            deltaAngleL -= 360;

        globalAngleL += deltaAngleL;
        lastAngleL = angleL;

        double deltaAngleR = angleR - lastAngleR;

        if (deltaAngleR < -180)
            deltaAngleR += 360;
        else if (deltaAngleR > 180)
            deltaAngleR -= 360;

        globalAngleR += deltaAngleR;
        lastAngleR = angleR;

        double angle, deltaAngle, globalAngle;
        if (rotationAxis == RotationAxis.LEFT) {
            angle =angleL;
            deltaAngle = deltaAngleL;
            globalAngle =globalAngleL;
        } else if (rotationAxis == RotationAxis.RIGHT) {
            angle =angleR;
            deltaAngle = deltaAngleR;
            globalAngle =globalAngleR;
        } else {
            angle = (angleL+angleR) / 2;
            deltaAngle = (deltaAngleL+deltaAngleR) / 2;
            globalAngle = (globalAngleL + globalAngleR) / 2;
        }
        //RobotLog.ii(TAG_TD, "getAngle() angles firstAngle=" + angles.firstAngle + " secondAngle=" + angles.secondAngle + " thirdAngle=" + angles.thirdAngle);
        RobotLog.ii(TAG_TD, "getAngle() angles=" + angle + " deltaAngle=" + deltaAngle + " globalAngle=" + globalAngle);
        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public double turnUsingPID(double degrees, double power, RotationAxis rotationAxis) throws InterruptedException {
        return turnUsingPID(degrees,power,0,0f,rotationAxis);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public double turnUsingPID(double degrees, double power, double power2, float maxDistanceInch, RotationAxis rotationAxis) throws InterruptedException {
        return turnUsingPID(degrees,power,power2,maxDistanceInch,rotationAxis,false,0);
    }

    public double turnUsingPID(double degrees, double power, double power2, float maxDistanceInch,
                               RotationAxis rotationAxis, boolean accurateTurn,int timeoutMillis) throws InterruptedException {

        double average,ticks;
        double tolerance = 1;
        long onTargetTime=0, stablingTime = 0;
        boolean onTarget = false;
        double maxRange = Math.abs(degrees);
        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        RobotLog.ii(TAG_TD, "turnUsingPID() start degrees=" + degrees + " power=" + power);
        // restart imu angle tracking.
        //resetAngle(rotationAxis);
        resetAngle();
        resetPosition();
        ticks = inchToTick(maxDistanceInch);

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);
        RobotLog.ii(TAG_TD, "turnUsingPID() 1");

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        if (accurateTurn){
            tolerance = 0.05;
            stablingTime = 100;
            maxRange += 10;
        }
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, maxRange);
        pidRotate.setOutputRange(0, Math.abs(power));
        pidRotate.setTolerance(tolerance);
        pidRotate.enable();
        RobotLog.ii(TAG_TD, "turnUsingPID() 2");

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        WaitLinear lp = new WaitLinear(op);

        if (degrees < 0)
        {
            RobotLog.ii(TAG_TD, "turnUsingPID() 3");
            do
            {
                if (power2 > 0f) {
                    average = getEncoder();
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }
                power = pidRotate.performPID(getAngle(RotationAxis.CENTER)); // power will be - on right turn.
                if (rotationAxis == RotationAxis.LEFT){
                    setPower(Math.signum(power)*power2, power);
                } else if (rotationAxis == RotationAxis.RIGHT){
                    setPower(-power, -Math.signum(power)*power2);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(-power, power);
                }
//                odometryLocalization.update();
                RobotLog.ii(TAG_TD, "turnUsingPID() right power=" + power);

                /*
                if (false == onTarget) {
                    if (pidRotate.onTarget()) {
                        if (!accurateTurn || stablingTime == 0) {
                            break;
                        } else {
                            onTarget = true;
                            onTargetTime = System.currentTimeMillis();
                        }
                    }
                } else if (!accurateTurn || (System.currentTimeMillis() - onTargetTime) >= stablingTime){
                    break;
                }
                */
                timeStamp = System.currentTimeMillis();
            } while (!pidRotate.onTarget() && op.opModeIsActive() && (timeoutMillis <=0 || timeStamp - startTimestamp < timeoutMillis));
        }
        else {  // left turn.
            do {
                if (power2 > 0f) {
                    average = Math.abs(getEncoder());
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }

                power = pidRotate.performPID(getAngle(RotationAxis.CENTER)); // power will be + on left turn.
                //setPower(-power, power);
                if (rotationAxis == RotationAxis.LEFT) {
                    setPower(power2, power);
                } else if (rotationAxis == RotationAxis.RIGHT) {
                    setPower(-power, -power2);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(-power, power);
                }
//                odometryLocalization.update();
                RobotLog.ii(TAG_TD, "turnUsingPID() left power=" + power);

                /*
                if (false == onTarget) {
                    if (pidRotate.onTarget()) {
                        if (!accurateTurn || stablingTime == 0) {
                            break;
                        } else {
                            onTarget = true;
                            onTargetTime = System.currentTimeMillis();
                        }
                    }
                } else if (!accurateTurn || (System.currentTimeMillis() - onTargetTime) >= stablingTime){
                    break;
                }
                */
                timeStamp = System.currentTimeMillis();
            } while (!pidRotate.onTarget() && op.opModeIsActive() && (timeoutMillis <=0 || timeStamp - startTimestamp < timeoutMillis));
        }

        RobotLog.ii(TAG_TD, "turnUsingPID() done loop");
        // turn the motors off.
        setPower(0, 0);
//        odometryLocalization.update();

        rotation = getAngle(RotationAxis.CENTER);

        // wait for rotation to stop.
        lp.waitMillis(100);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii(TAG_TD, "turnUsingPID() done degrees=" + degrees + " power=" + power + " rotation=" + rotation);
        //if (odometryEnabled)
//        odometryLocalization.showValues();

        return rotation;
    }

}

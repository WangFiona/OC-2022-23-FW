package overcharged.linear.components;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import overcharged.components.Drive;
import overcharged.components.Encoder;
import overcharged.components.EncoderController;
import overcharged.components.OcBnoGyro;
import overcharged.linear.components.OcEncoder;
import overcharged.components.OcGyro;
import overcharged.components.OcMotor;
import overcharged.components.RotationAxis;
import overcharged.components.TankDrive;
import overcharged.components.TurnType;
import overcharged.linear.util.WaitLinear;

import overcharged.pid.Controller;
import overcharged.util.Stalled;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static overcharged.config.RobotConstants.TAG_TD;

/**
 * This class is for Overcharged autonomous
 * Updated by Parthiv Nair on 9/17/2018.
 */
public class TankDriveLinearTest2
        extends TankDrive
{
    public final OcGyro gyroSensor;

    private OcMotorsLinear motors;

    private LinearOpMode op;

    WaitLinear lp;

    public EncoderController encoder;

    //Global switch to disable odometry
    boolean odometryEnabled = false;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.10f;
    private final static float POWER_START = 0.17f;

    Controller pidRotate, pidDrive;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction, rotation;
    private final static float LEFT_POWER_ADJ = 0.84f;


    /*
    An obvious question is how did we arrive at .003 for the P value on the turn PID controller. In the case of moving from a non-zero error to zero error, we took the maximum error value, 90, and the power level we want applied at max error, .30 (30%). We divided the power by the max error ( .30 / 90 = .003 ) to determine P.
    Often with just a P value alone, the robot may stall out before completing the turn because the PID controller reduces the power below the level which will move the robot. To fix this, we add some I (integral) value. The I value will compensate for P not reaching the setpoint and start adding power until the robot completes the turn. A good starting I value is P / 100. You can adjust I to get the turn completed in a timely fashion. Note that these values are optimal for a 90 degree turn with 30% power. They will not work as well for other angles or power levels. You would have to compute new values for other angle/power combinations. Here is the above example modified to compute the P and I values for any angle/power combination input to the rotate() function.
    In the case of driving straight, the target and error are the same at the start so the error is zero. So we have to determine how much correction we want to apply for how much error. Experimentation showed that .05 (5%) correction power for 1 degree of error worked well, correcting the error without overshooting too much (wandering). 1 / .05 = .05 for P.
    As always, you can do these calculations to determine a starting P and I values and then adjust then to tune actual robot behavior.
    In most simple cases only a P value is needed. I may be needed if you can't find a P value that reaches the setpoint with out overshooting. A discussion of using the D value is beyond the scope of this lesson.
     */
    public TankDriveLinearTest2 (OcMotor driveLeftFront,
                                 OcMotor driveLeftBack,
                                 OcMotor driveRightFront,
                                 OcMotor driveRightBack,
                                 OcGyro gyro,
                                 Encoder left, Encoder right,
                                 EncoderController encoder)
    {
        super(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);

        this.gyroSensor = gyro;
        this.encoder = encoder;
        this.motors = new OcMotorsLinear(this.motorsArray);
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
//        pidRotate = new PIDController(.003, .00003, 0);
        pidRotate = new Controller(0.08, 0, 0.08);

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

    public void moveToEncoderInch(int distanceInch,
                                  float maxPower,
                                  int timeoutMillis,
                                  boolean isAccelerate,
                                  boolean isAdjust,
                                  boolean isBreak)

            throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(tick,
                maxPower,
                timeoutMillis,
                isAccelerate,
                isBreak,
                isAdjust,
                null);
    }

    public void moveToEncoderInch(
            int distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isAccelerate,
            boolean isAdjust,
            boolean isBreak,
            MoveAction... actions)
            throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        moveToEncoder(tick,
                maxPower,
                timeoutMillis,
                isAccelerate,
                isBreak,
                isAdjust,
                actions);
    }

    public void moveToEncoder(int tick,
                              float maxPower,
                              int timeoutMillis,
                              boolean isAccelerate,
                              boolean isAdjust,
                              boolean isBreak,
                              MoveAction[] actions)
            throws InterruptedException
    {
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        resetPosition();
        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_TO_POSITION);
        setTargetPosition2(tick);

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis)) {

            double average = getEncoder();
            double delta = tick - average;
            if (Math.abs(delta) < 30)
            {
                // done
                break;
            }

            double basePower;
            if (!isAccelerate)
            {
                basePower = Math.signum(delta) * (maxPower);
            }
            else
            {
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    basePower = (average/1600f) + Math.signum(tick) * (POWER_START);
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    basePower = (delta/3200f) + Math.signum(delta) * (POWER_STOP);
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

            } // if

            //op.telemetry.addData("power", Float.toString(basePower));
            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(op);
                stop2();
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
            }
            else {
                if (isAdjust) {
                    adjustPower(basePower);
                } else {
                    setPower(basePower);
                }
            }

            if (actions != null) {
                for (MoveAction action : actions) {
                    if (action == null) {
                        continue;
                    }

                    if (Math.abs(average) > Math.abs(action.distanceInTick())) {
                        action.perform();
                    }
                }
            }

            op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            stop2();
        }

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.ee(TAG_TD, "Thread interrupted");
        } else if (op.isStopRequested()) {
            RobotLog.ee(TAG_TD,"Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        if (actions != null) {
            for (MoveAction action : actions) {
                if (action == null) {
                    continue;
                }

                action.stop();
                RobotLog.ii(TAG_TD, "Actions " + action + " stopped");
            }
            op.idle();
        }

        double inches = tickToInch(getEncoder());
        RobotLog.ii(TAG_TD, "Moved " + inches + " inches");
        RobotLog.ii(TAG_TD, "Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }

    public void moveToTime (
            float power,
            int timeoutMillis)
            throws InterruptedException
    {
        final float p = Range.clip(power, -1f, 1f);

        stop2();

        setMode2(RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior2(ZeroPowerBehavior.FLOAT);

        WaitLinear lp = new WaitLinear(op);
        lp.waitMillis(timeoutMillis,
                new WaitLinear.WakeUp() {
                    public boolean isWakeUp() {
                        // in case skipping
                        setPower(p);
                        return false;
                    }
                });

        stop2();
        setMode2(RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior2(ZeroPowerBehavior.BRAKE);
        op.idle();
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

    public static float GetOppositeSide(float degrees, float distance)
    {
        //Sin theta = Opposite/Hypotenuse. distance=Hypotenuse
        double radians = Math.toRadians(degrees);
        return new Double(Math.sin(radians) * distance).floatValue();
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
     * course adjust the power
     * @param p power to adjust with
     */
    private void adjustPower (float p, float adjust) {
        // with RUN_TO_POSITION sign of power is ignored
        // we have to clip so that (1 + adjust) and (1 - adjust) have the same sign
        adjust = Range.clip(adjust, -1, 1);
        // BNO055 positive for left turn
        float pwrl = p * (1 - adjust);
        float pwrr = p * (1 + adjust);

        float max = Math.max(Math.abs(pwrl), Math.abs(pwrr));
        if (max > 1f) {
            // scale to 1
            pwrl = pwrl / max;
            pwrr = pwrr / max;
        }
        setPower(pwrl,
                pwrr);
    }

    /**
     * course adjust the power
     * @param p power to adjust with
     */
    private void adjustPower (double p) {
        double adjust = Math.signum(p) * gyroSensor.adjustDirection();
        // with RUN_TO_POSITION sign of power is ignored
        // we have to clip so that (1 + adjust) and (1 - adjust) have the same sign
        adjust = Range.clip(adjust, -1, 1);
        // BNO055 positive for left turn
        double pwrl = p * (1 - adjust);
        double pwrr = p * (1 + adjust);

        double max = Math.max(Math.abs(pwrl), Math.abs(pwrr));
        if (max > 1f) {
            // scale to 1
            pwrl = pwrl / max;
            pwrr = pwrr / max;
        }
        setPower(pwrl,
                pwrr);
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = gyroSensor.getAngularOrientation();
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = gyroSensor.getAngularOrientation();

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        //RobotLog.ii(TAG_TD, "getAngle() angles firstAngle=" + angles.firstAngle + " secondAngle=" + angles.secondAngle + " thirdAngle=" + angles.thirdAngle);
        RobotLog.ii(TAG_TD, "getAngle() angles=" + angles.firstAngle + " deltaAngle=" + deltaAngle + " globalAngle=" + globalAngle);
        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void turnUsingPID(double degrees, double power, RotationAxis rotationAxis) throws InterruptedException {
        turnUsingPID(degrees,power,0,0f,rotationAxis);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void turnUsingPID(double degrees, double power, double power2, float maxDistanceInch, RotationAxis rotationAxis) throws InterruptedException {
        double average,ticks;

        RobotLog.ii(TAG_TD, "turnUsingPID() start degrees=" + degrees + " power=" + power);
        // restart imu angle tracking.
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

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
        RobotLog.ii(TAG_TD, "turnUsingPID() 2");

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        WaitLinear lp = new WaitLinear(op);

        if (degrees < 0)
        {
//            // On right turn we have to get off zero first.
//            while (op.opModeIsActive() && getAngle() == 0)
//            {
//                setPower(power, -power);
//                lp.waitMillis(50);
//            }

            RobotLog.ii(TAG_TD, "turnUsingPID() 3");
            do
            {
                if (power2 > 0f) {
                    average = getEncoder();
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                if (rotationAxis == RotationAxis.LEFT){
                    setPower(Math.signum(power)*power2, power);
                } else if (rotationAxis == RotationAxis.RIGHT){
                    setPower(-power, -Math.signum(power)*power2);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(-power, power);
                }
                RobotLog.ii(TAG_TD, "turnUsingPID() right power=" + power);
            } while (!pidRotate.onTarget() && op.opModeIsActive());
        }
        else    // left turn.
            do
            {
                if (power2 > 0f) {
                    average = Math.abs(getEncoder());
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }

                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //setPower(-power, power);
                if (rotationAxis == RotationAxis.LEFT){
                    setPower(power2, power);
                } else if (rotationAxis == RotationAxis.RIGHT){
                    setPower(-power, -power2);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(-power, power);
                }
                RobotLog.ii(TAG_TD, "turnUsingPID() left power=" + power);
            } while (!pidRotate.onTarget() && op.opModeIsActive());

        RobotLog.ii(TAG_TD, "turnUsingPID() done loop");
        // turn the motors off.
        setPower(0, 0);

        rotation = getAngle();

        // wait for rotation to stop.
        lp.waitMillis(100);

        // reset angle tracking on new heading.
        resetAngle();
        RobotLog.ii(TAG_TD, "turnUsingPID() done degrees=" + degrees + " power=" + power + " rotation=" + rotation);
        if (odometryEnabled) RobotLog.ii(TAG_TD, " Encoder Inches=" + encoder.getDistanceToString() + " Encoder=" + encoder.getCurrentPositionString() + " Drift" + encoder.getDrift());
    }

    /**
     * Move the robot and also ensure that we are not colliding with our partner
     * @param distanceInch the distance to move in inches
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isBreak whether to break
     * @param adjustLeftPower
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @throws InterruptedException
     */
    public void moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBreak,
            boolean adjustLeftPower,
            boolean isAccelerate)
            throws InterruptedException
    {
        moveToEncoderInchUsingPID(distanceInch,
                maxPower,
                timeoutMillis,
                isBreak,
                adjustLeftPower,
                isAccelerate,
                null,
                (MoveAction) null);
    }
    public void moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBreak,
            MoveAction... actions)
            throws InterruptedException
    {
        moveToEncoderInchUsingPID(distanceInch,
                maxPower,
                timeoutMillis,
                isBreak,
                true,
                false,
                null,
                actions);
    }

    /**
     * Move the robot and also ensure that we are not colliding with our partner
     * @param distanceInch the distance to move in inches
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isBrake whether to break
     * @param adjustLeftPower
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param sensorDistance if specified (not null) then we detect collision
     * @param actions
     * @throws InterruptedException
     */
    public void moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBrake,
            boolean adjustLeftPower,
            boolean isAccelerate,
            DistanceSensor sensorDistance,
            MoveAction... actions)
            throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;
        float orgPower = maxPower;
        float left_power_adj = 1f;
        double basePower,basePowerL,basePowerR;
        double ratio = 1;
        int targetEncoderValue = (int) (distanceInch * OcEncoder.COUNTS_PER_INCH);

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);
        //maxPower = 0.1f;
        if (adjustLeftPower && orgPower > 0.6) {
            left_power_adj = LEFT_POWER_ADJ;
        }

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        resetPosition();
        // is RUN_TO_POSITION
        setTargetPosition2(tick);
        setMode2(RunMode.RUN_TO_POSITION);


        // Set up parameters for driving in a straight line.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, orgPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        double average = getEncoder();
        boolean forward = (distanceInch >= 0);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID starting encoder value=" + average);
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis)) {
            if (odometryEnabled) {
                if (encoder.getCurrentPosition() > targetEncoderValue) {
                    RobotLog.ii(TAG_TD, encoder.getCurrentPositionString());
                    break;
                }
            }

            if (sensorDistance != null) {
                double distance = sensorDistance.getDistance(DistanceUnit.INCH);
                RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID: sonar distance inches=" + distance);
                if (distance <= 10) {
                    RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID: Collision detected so waiting");
                    setPower(0f, 0f);
                    lp.waitMillis(500);
                }
            }

            average = getEncoder();
            double delta = tick - average;
            if (Math.abs(delta) < 30)
            {
                // done
                break;
            } else if (isBrake && maxPower > 0.7f && tickToInch(Math.abs(delta)) < 5) {
                // slow down before stopping
                maxPower *= 0.9;
            }

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());
            RobotLog.ii(TAG_TD, "imu heading=" + lastAngles.firstAngle + " global heading=" + globalAngle + " correction=" + correction + " turn rotation=" + rotation);

            //op.telemetry.addData("power", Float.toString(basePower));
            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(op);
                stop2();
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
            }
            else {
                basePower = maxPower;
                if (isAccelerate) {
                    basePower = getAccelerationPower(tick, average, delta, maxPower);
                    ratio = basePower/maxPower;
                }
                basePowerL = Math.signum(delta) * (basePower*left_power_adj - correction*ratio);
                //basePowerL = Range.clip(basePowerL, -maxPower, maxPower);
                basePowerR = Math.signum(delta) * (basePower + correction*ratio);
                //basePowerR = Range.clip(basePowerR, -maxPower, maxPower);
                RobotLog.ii(TAG_TD, "basePowerL=" + basePowerL + " basePowerR=" + basePowerR);
                if (forward)
                    setPower(basePowerL, basePowerR);
                else
                    setPower(basePowerR, basePowerL);
            }

            if (actions != null) {
                for (MoveAction action : actions) {
                    if (action == null) {
                        continue;
                    }

                    if (Math.abs(average) > Math.abs(action.distanceInTick())) {
                        action.perform();
                    }
                }
            }

            //op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBrake) {
            //stop2();
            setPower(0, 0);
        }

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.ee(TAG_TD, "Thread interrupted");
        } else if (op.isStopRequested()) {
            RobotLog.ee(TAG_TD, "Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        if (actions != null) {
            for (MoveAction action : actions) {
                if (action == null) {
                    continue;
                }

                action.stop();
                RobotLog.ii(TAG_TD, "Actions " + action + " stopped");
            }
            op.idle();
        }

        double average2 = getEncoder();
        double inches = tickToInch(average);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID ending encoder value=" + average2 + " Before stopping=" +average);
        RobotLog.ii(TAG_TD, "Moved " + inches + " inches" + tick +"ticks");
        if (odometryEnabled) RobotLog.ii(TAG_TD, " Encoder Inches=" + encoder.getDistanceToString() + " Encoder=" + encoder.getCurrentPositionString() + " Drift" + encoder.getDrift());
        RobotLog.ii(TAG_TD, "Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }

    private double getAccelerationPower(double tick, double average, double delta, double maxPower) {
        double basePower = maxPower;
        if (Math.abs(average) < Math.abs(tick / 4)) {
            // ramp up for 1/4 of distance
            // minimum power is START_POWER
            // base is 1200
            //basePower = (average / 1200f) + Math.signum(tick) * POWER_START;
            basePower = (1.0*Math.abs(average) / 500.0) + POWER_START;
        } else if (Math.abs(average) > Math.abs((tick * 3)/ 4)) {
            // ramp down for 3/4 of distance
            // minimum power is START_POWER
            // base is 3600
            //basePower = (delta / 3600f) + Math.signum(delta) * POWER_STOP;
            basePower = (1.0*Math.abs(delta) / 1600.0) + POWER_STOP;
        }

        basePower = Range.clip(basePower, 0, maxPower);
        return basePower;
    }

    /**
     * Move the robot and also ensure that we are not over or under shooting
     * @param distanceInch the distance to move in inches
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isBrake whether to break
     * @param adjustLeftPower
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @param sensorDistance if specified (not null) then we detect the object in from of us
     * @param actions
     * @throws InterruptedException
     */
    public void moveToEncoderInchUsingPIDAndSensor(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBrake,
            boolean adjustLeftPower,
            boolean isAccelerate,
            DistanceSensor sensorDistance,
            double startSensingDistance,
            MoveAction... actions)
            throws InterruptedException
    {
        int tick = inchToTick(distanceInch);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;
        float orgPower = maxPower;
        float left_power_adj = 1f;
        int targetEncoderValue = (int) (distanceInch * OcEncoder.COUNTS_PER_INCH);

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);
        //maxPower = 0.1f;
        if (adjustLeftPower && orgPower > 0.6) {
            left_power_adj = LEFT_POWER_ADJ;
        }

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        resetPosition();
        // is RUN_TO_POSITION
        setTargetPosition2(tick);
        setMode2(RunMode.RUN_TO_POSITION);


        // Set up parameters for driving in a straight line.
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, orgPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        double average = getEncoder();
        boolean forward = (distanceInch >= 0);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor starting encoder value=" + average);
        double stopDistance = 19.0;
        double adjustDistance = stopDistance + 1.5;
        boolean keepGoing = true;
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis) && keepGoing) {
            if (odometryEnabled) {
                if (encoder.getCurrentPosition() > targetEncoderValue) {
                    RobotLog.ii(TAG_TD, "Reached odometry limit " + encoder.getCurrentPositionString());
                    keepGoing = false;
                }
            }

            average = getEncoder();
            double delta = tick - average;
            double deltaInch = tickToInch(Math.abs(delta));
            RobotLog.ii(TAG_TD, "distance left deltaInch=" + deltaInch + " startSensingDistance=" + startSensingDistance);
            if (Math.abs(delta) < 30)
            {
                // done
                keepGoing = false;
                RobotLog.ii(TAG_TD, "Reached distance limit");
            } else if (isBrake && maxPower > 0.7f && deltaInch < 5) {
                // slow down before stopping
                maxPower *= 0.9;
            }
            if (sensorDistance != null) {
                if (deltaInch <= startSensingDistance) {
                    double distance = sensorDistance.getDistance(DistanceUnit.INCH);
                    RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: sensor distance inches=" + distance);
                    if (distance <= stopDistance) {
                        if (keepGoing) {
                            RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: reached sensor limit stop early");
                            keepGoing = false;
                        }
                    } else if (distance <= adjustDistance && !keepGoing) {
                        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: did not reach reach sensor limit so keep going");
                        keepGoing = true;
                    }
                }
            }
            if (!keepGoing) break;

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());
            RobotLog.ii(TAG_TD, "imu heading=" + lastAngles.firstAngle + " global heading=" + globalAngle + " correction=" + correction + " turn rotation=" + rotation);

            //op.telemetry.addData("power", Float.toString(basePower));
            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(op);
                stop2();
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
            }
            else {
                double basePower = maxPower;
                if (isAccelerate) {
                    basePower = getAccelerationPower(tick, average, delta, maxPower);
                }
                double basePowerL = Math.signum(delta) * (basePower*left_power_adj - correction*basePower/maxPower);
                //basePowerL = Range.clip(basePowerL, -maxPower, maxPower);
                double basePowerR = Math.signum(delta) * (basePower + correction*basePower/maxPower);
                //basePowerR = Range.clip(basePowerR, -maxPower, maxPower);
                RobotLog.ii(TAG_TD, "basePowerL=" + basePowerL + " basePowerR=" + basePowerR);
                if (forward)
                    setPower(basePowerL, basePowerR);
                else
                    setPower(basePowerR, basePowerL);
            }

            if (actions != null) {
                for (MoveAction action : actions) {
                    if (action == null) {
                        continue;
                    }

                    if (Math.abs(average) > Math.abs(action.distanceInTick())) {
                        action.perform();
                    }
                }
            }

            //op.idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBrake) {
            //stop2();
            setPower(0, 0);
        }

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.ee(TAG_TD, "Thread interrupted");
        } else if (op.isStopRequested()) {
            RobotLog.ee(TAG_TD, "Stop requested");
        }

        // is RUN_TO_POSITION
        setMode2(RunMode.RUN_USING_ENCODER);

        // stop actions
        if (actions != null) {
            for (MoveAction action : actions) {
                if (action == null) {
                    continue;
                }

                action.stop();
                RobotLog.ii(TAG_TD, "Actions " + action + " stopped");
            }
            op.idle();
        }

        double average2 = getEncoder();
        double inches = tickToInch(average);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor ending encoder value=" + average2 + " Before stopping=" +average);
        RobotLog.ii(TAG_TD, "Moved " + inches + " inches" + tick +"ticks");
        if (odometryEnabled) RobotLog.ii(TAG_TD, " Encoder Inches=" + encoder.getDistanceToString() + " Encoder=" + encoder.getCurrentPositionString() + " Drift" + encoder.getDrift());
        RobotLog.ii(TAG_TD, "Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();
    }
}
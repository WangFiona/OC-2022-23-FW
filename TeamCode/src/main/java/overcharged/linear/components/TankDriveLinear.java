package overcharged.linear.components;


import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import overcharged.R;
import overcharged.components.Drive;
import overcharged.components.Encoder;
import overcharged.components.NavxImu;
import overcharged.components.OcGyro;
import overcharged.components.OcGyro2;
import overcharged.components.OcMotor;
import overcharged.components.RotationAxis;
import overcharged.components.TankDrive;
import overcharged.components.TurnType;
import overcharged.components.Cup;
import overcharged.config.RobotConstants;
import overcharged.linear.util.WaitLinear;
import overcharged.odometry.Localization;
import overcharged.pid.Controller;
import overcharged.util.Stalled;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static overcharged.config.RobotConstants.TAG_TD;

/**
 * This class is for Overcharged autonomous
 * Updated by Parthiv Nair on 9/17/2018.
 */
public class
TankDriveLinear
        extends TankDrive
{
    //public final OcGyro gyroSensor;
    public final OcGyro2 gyroSensor;

    private OcMotorsLinear motors;

    private LinearOpMode op;

    WaitLinear lp;
    private ElapsedTime deltaTimeMeasurement = new ElapsedTime();

    public Localization odometryLocalization;

    //Global switch to disable odometry
    boolean odometryEnabled = false;
    boolean yesResetAngle=true;

    /** constant for ramping up robot */
    private final static float POWER_STOP = 0.10f;
    private final static float POWER_START = 0.17f;

    Controller           pidRotate, pidDrive;
    PIDFController pidRotateNew;
    //Orientation lastAngles = new Orientation();
    double                  lastAngle = 0;
    double                  globalAngle;
    double currentAngle=0;
    double                  power = .30, correction, rotation;
    private final static float LEFT_POWER_ADJ =1; // 0.76f;


    /*
    An obvious question is how did we arrive at .003 for the P value on the turn PID controller. In the case of moving from a non-zero error to zero error, we took the maximum error value, 90, and the power level we want applied at max error, .30 (30%). We divided the power by the max error ( .30 / 90 = .003 ) to determine P.
    Often with just a P value alone, the robot may stall out before completing the turn because the PID controller reduces the power below the level which will move the robot. To fix this, we add some I (integral) value. The I value will compensate for P not reaching the setpoint and start adding power until the robot completes the turn. A good starting I value is P / 100. You can adjust I to get the turn completed in a timely fashion. Note that these values are optimal for a 90 degree turn with 30% power. They will not work as well for other angles or power levels. You would have to compute new values for other angle/power combinations. Here is the above example modified to compute the P and I values for any angle/power combination input to the rotate() function.
    In the case of driving straight, the target and error are the same at the start so the error is zero. So we have to determine how much correction we want to apply for how much error. Experimentation showed that .05 (5%) correction power for 1 degree of error worked well, correcting the error without overshooting too much (wandering). 1 / .05 = .05 for P.
    As always, you can do these calculations to determine a starting P and I values and then adjust then to tune actual robot behavior.
    In most simple cases only a P value is needed. I may be needed if you can't find a P value that reaches the setpoint with out overshooting. A discussion of using the D value is beyond the scope of this lesson.
     */
    public TankDriveLinear (OcMotor driveLeftFront,
                            OcMotor driveLeftBack,
                            OcMotor driveRightFront,
                            OcMotor driveRightBack,
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
        pidRotate = new Controller(0.075, 0, 0.04);
        //pidRotate = new Controller(0.08, 0, 0.04);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        //pidDrive = new PIDController(.04, 0, 0.005);
        // competitoin pidDrive = new PIDController(.03, 0, 0.005);
        pidDrive = new Controller(0.01, 0, 0.004);
        //pidDrive = new Controller(0.03, 0, 0.005);

        RobotLog.ii(TAG_TD, "initialized");

    }

    public void yesReset(boolean reset){
        if(reset==false){yesResetAngle=false;}
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

    public static float GetOppositeSide(float degrees, float distance)
    {
        //Sin theta = Opposite/Hypotenuse. distance=Hypotenuse
        double radians = Math.toRadians(degrees);
        return new Double(Math.sin(radians) * distance).floatValue();
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
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        //lastAngles = gyroSensor.getLAngularOrientation();
        lastAngle = gyroSensor.getHeading();
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    public double getAngle(RotationAxis rotationAxis)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        //Orientation angles = gyroSensor.getAngularOrientation();
        double angle;
        //angleL = gyroSensor.getLFirstAngle();
        //angleR = gyroSensor.getRFirstAngle();
        angle = gyroSensor.getHeading();

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngle = angle;

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
        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        double zeroorprevious = 0;
        if(yesResetAngle==false){zeroorprevious=getAngle(RotationAxis.CENTER);}
        double maxRange = Math.abs(degrees-zeroorprevious);
        RobotLog.ii(TAG_TD, "turnUsingPID() get angle=" + zeroorprevious);


        double startingangle = getAngle(RotationAxis.CENTER);

        RobotLog.ii(TAG_TD, "turnUsingPID() start degrees=" + degrees + "starting angle=" + startingangle + " power=" + power);        // restart imu angle tracking.
        //resetAngle(rotationAxis);
        if(yesResetAngle==true){resetAngle();}
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
        pidRotate.setSetpoint(degrees-zeroorprevious);
        pidRotate.setInputRange(0, maxRange);
        pidRotate.setOutputRange(0, Math.abs(power));
        pidRotate.setTolerance(tolerance);
        pidRotate.enable();
        RobotLog.ii(TAG_TD, "turnUsingPID() 2");

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.
        WaitLinear lp = new WaitLinear(op);

        if (degrees < zeroorprevious)
        {
            RobotLog.ii(TAG_TD, "turnUsingPID() 3");
            do
            {
                startingangle = getAngle(RotationAxis.CENTER);
                if (power2 > 0f) {
                    average = getEncoder();
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }

                power = pidRotate.performPID(getAngle(RotationAxis.CENTER)-zeroorprevious); // power will be - on right turn.
                if (rotationAxis == RotationAxis.LEFT){
                    setPower(power, Math.signum(power)*power2);
                } else if (rotationAxis == RotationAxis.RIGHT){
                    setPower(-Math.signum(power)*power2, -power);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(power, -power);
                }
//                odometryLocalization.update();
                RobotLog.ii(TAG_TD, "turnUsingPID() right power=" + power + "starting angle" + startingangle);

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
                startingangle = getAngle(RotationAxis.CENTER);
                if (power2 > 0f) {
                    average = Math.abs(getEncoder());
                    if (average > ticks) {
                        power2 = 0f;
                    }
                }

                power = pidRotate.performPID(getAngle(RotationAxis.CENTER)-zeroorprevious); // power will be + on left turn.
                //setPower(-power, power);
                if (rotationAxis == RotationAxis.LEFT) {
                    setPower(power, power2);
                } else if (rotationAxis == RotationAxis.RIGHT) {
                    setPower(-power2, -power);
                } else if (rotationAxis == RotationAxis.CENTER) {
                    setPower(power, -power);
                }
//                odometryLocalization.update();
                RobotLog.ii(TAG_TD, "turnUsingPID() left power=" + power + "starting angle" + startingangle);

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
        if(yesResetAngle==true){resetAngle();}
        RobotLog.ii(TAG_TD, "turnUsingPID() done degrees=" + degrees + " power=" + power + " rotation=" + rotation);
        //if (odometryEnabled)
//        odometryLocalization.showValues();

        return rotation;
    }

    public void curveTurnUsingPID(double degrees, double power, double a, boolean forward, boolean isBrake) throws InterruptedException {
        resetPosition();
        double basePower=power;
        double ratio = 1;
        boolean left=true;

        double r=a;
        r= (degrees-getAngle(RotationAxis.CENTER)==90 || degrees-getAngle(RotationAxis.CENTER)==-90) ? a : a/Math.sin(Math.toRadians(degrees-getAngle(RotationAxis.CENTER)));
        r = Math.abs(r);
        RobotLog.ii(TAG_TD, "curveturn: a=" + a + " getangle=" + getAngle(RotationAxis.CENTER) + " sin=" + Math.sin(Math.toRadians(degrees-getAngle(RotationAxis.CENTER))));

        double innerPower = power*(r-4.52756)/(r+4.52756);
        double inToTick = 537.6 / (Math.PI * 1.88976);

        double originalAngle = getAngle(RotationAxis.CENTER);
        double innerOgDis = innerAverageEncoder();
        double outerOgDis = outerAverageEncoder();
        double disInner = innerAverageEncoder()-innerOgDis;
        double disOuter = outerAverageEncoder()-outerOgDis;
        double xInner = (inToTick*(2*Math.PI*(r-4.52756)))/360;
        double xOuter = (inToTick*(2*Math.PI*(r+4.52756)))/360;
        double xDiff = Math.abs(xOuter-xInner);
        double disDiff = Math.abs(disOuter-disInner);
        double traveled = disDiff/xDiff;

        /*double originalDis = averageEncoder();
        double originalAngle = getAngle(RotationAxis.CENTER);
        double dis = averageEncoder()-originalDis; //encoder value
        double x = (inToTick*(2*Math.PI*r))/360;
        double traveled = dis/x;*/
        double turned = 0;
        RobotLog.ii(TAG_TD, "curveturn: xInner=" + xInner + " xOuter=" + xOuter
                + " disInner=" + disInner + " disOuter" + disOuter + " r=" + r);

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        int sign=1;
        if(getAngle(RotationAxis.CENTER)-degrees>0)
            sign =-1;

        int forwardSign=1;
        if(forward==false)
            forwardSign=-1;

        if((forward && sign>0) || (!forward && sign<0))
            left=false;

        innerPower *= Math.signum(forwardSign);
        RobotLog.ii(TAG_TD, "curveturn: startingangle=" + getAngle(RotationAxis.CENTER));

        while(op.opModeIsActive() && sign*getAngle(RotationAxis.CENTER)<sign*degrees) {
            disInner = innerAverageEncoder()-innerOgDis;
            disOuter = outerAverageEncoder()-outerOgDis;
            xInner = (inToTick*(2*Math.PI*(r-4.52756)))/360;
            xOuter = (inToTick*(2*Math.PI*(r+4.52756)))/360;
            xDiff = Math.abs(xOuter-xInner);
            disDiff = Math.abs(disOuter-disInner);
            traveled = disDiff/xDiff;

            turned = Math.abs(getAngle(RotationAxis.CENTER)-originalAngle);
            RobotLog.ii(TAG_TD, "curveturn: target angle=" + degrees + " traveled=" + traveled + " turned=" + turned);

            correction = pidDrive.performPID(traveled-turned);

        /*if (isAccelerate) {
            basePower = getAccelerationPower(tick, average, delta, maxPower);
            ratio = basePower/power;
        }*/
            //power = Math.signum(power) * (power - correction * ratio);
            //innerPower = Math.signum(delta) * (basePower + correction*ratio);

            if(forward){
                power = Math.signum(forwardSign)*(basePower + correction * ratio);
            }
            else {
                power = Math.signum(forwardSign)*(basePower + correction * ratio);
            }

            if (left) {
                setPower(innerPower, power);
            }
            else {
                setPower(power, innerPower);
            }
            RobotLog.ii(TAG_TD, "curveturn: correction=" + correction + " power=" + power + " innerpower=" + innerPower);
        }

        if(isBrake)
            setPower(0,0);
        else
            setPower(power);
    }

    public int innerAverageEncoder(){
        int average = (driveLeftBack.getCurrentPosition()+driveLeftFront.getCurrentPosition())/2;
        return average;
    }

    public int outerAverageEncoder(){
        int average = (driveRightBack.getCurrentPosition()+driveRightFront.getCurrentPosition())/2;
        return average;
    }

    /**
     * Move the robot and also ensure that we are not colliding with our partner
     * @param distanceInch the distance to move in inches
     * @param maxPower maximum motor power
     * @param timeoutMillis maximum execution time for this function in milliseconds
     * @param isBrake whether to brake
     * @param adjustLeftPower
     * @param isAccelerate whether to ramp up and down. Use false for very short distance
     * @throws InterruptedException
     */
    public double moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBrake,
            boolean adjustLeftPower,
            boolean isAccelerate)
            throws InterruptedException
    {
        return moveToEncoderInchUsingPID(distanceInch,
                maxPower,
                timeoutMillis,
                isBrake,
                adjustLeftPower,
                isAccelerate,
                null,
                (MoveAction) null);
    }
    public double moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            int timeoutMillis,
            boolean isBrake,
            MoveAction... actions)
            throws InterruptedException
    {
        return moveToEncoderInchUsingPID(distanceInch,
                maxPower,
                timeoutMillis,
                isBrake,
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
    public double moveToEncoderInchUsingPID(
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
        return moveToEncoderInchUsingPID(
                distanceInch,
                maxPower,
                0,
                0,
                0,
                timeoutMillis,
                isBrake,
                adjustLeftPower,
                isAccelerate,
                sensorDistance,
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
    public double moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            double turnAngle,
            float turnPointY,
            float turnPointY2,
            int timeoutMillis,
            boolean isBrake,
            boolean adjustLeftPower,
            boolean isAccelerate,
            DistanceSensor sensorDistance,
            MoveAction... actions)
            throws InterruptedException
    {
        return moveToEncoderInchUsingPID(
                distanceInch,
                maxPower,
                0,
                0,
                0,
                timeoutMillis,
                isBrake,
                adjustLeftPower,
                isAccelerate,
                sensorDistance,
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
         * @param cup
         * @throws InterruptedException
         */
    public double moveToEncoderInchUsingPID(
            float distanceInch,
            float maxPower,
            double turnAngle,
            float turnPointY,
            float turnPointY2,
            int timeoutMillis,
            boolean isBrake,
            boolean adjustLeftPower,
            boolean isAccelerate,
            DistanceSensor sensorDistance,
            Cup cup,
            MoveAction... actions)
            throws InterruptedException
    {
        int tick = inchToTick(distanceInch), tick1 = inchToTick(turnPointY), tick2 = inchToTick(turnPointY2), deltaTick=0;
        int turn = 0;
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;
        float orgPower = maxPower;
        float left_power_adj = 1f;
        double basePower,basePowerL,basePowerR;
        double ratio = 1;
        double angle=0;
        if(yesResetAngle==false){
            angle=getAngle(RotationAxis.CENTER);
        }
        int targetEncoderValue = (int) (distanceInch * OcEncoder.COUNTS_PER_INCH);
        int stopDeltaTicks = 30;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);
        //maxPower = 0.1f;
        if (adjustLeftPower && orgPower > 0.6) {
            left_power_adj = LEFT_POWER_ADJ;
        }

        if (turnAngle != 0 && turnPointY != 0 && turnPointY2 != 0) {
            turn = 1;
            deltaTick = (int)(tick2*(1/Math.cos(Math.toRadians(turnAngle)) - 1));
            tick += 2*deltaTick;
            RobotLog.ii(TAG_TD, "turnAngle=" + turnAngle +"tick="+ tick + "tick1=" + tick1 + "tick2 " + tick2 + "deltaTick=" + deltaTick);
        }

        if (isAccelerate) {
            stopDeltaTicks = 10;
        }

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        resetPosition();
        //resetAngle();
        // is RUN_TO_POSITION
        setTargetPosition2(tick);
        setMode2(RunMode.RUN_TO_POSITION);


        // Set up parameters for driving in a straight line.
        pidDrive.reset();
        pidDrive.setSetpoint(angle);
        pidDrive.setOutputRange(0, orgPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        Stalled stalled = new Stalled();
        int average = getEncoder();
        boolean forward = (distanceInch >= 0);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID starting encoder value=" + average);
        while (op.opModeIsActive() &&
                // is RUN_TO_POSITION
                isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis)) {
//            if (odometryEnabled) {
//                if (odometryLocalization.getCurrentPosition() > targetEncoderValue) {
//                    RobotLog.ii(TAG_TD, odometryLocalization.getCurrentPositionString());
//                    break;
//                }
//            }

            if (sensorDistance != null) {
                double distance = sensorDistance.getDistance(DistanceUnit.INCH);
                RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID: sonar distance inches=" + distance);
                if (distance <= 10) {
                    RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID: Collision detected so waiting");
                    setPower(0f, 0f);
//                    odometryLocalization.update();
                    lp.waitMillis(500);
                }
            }

            average = getEncoder();
            int delta = tick - average;
            forward = (delta >= 0);
            if (Math.abs(delta) < stopDeltaTicks)
            {
                // done
                break;
            } else if (cup != null && cup.isCollected()) {
                break;
            } else if (isBrake && maxPower > 0.7f && tickToInch(Math.abs(delta)) < 5) {
                // slow down before stopping
                maxPower *= 0.9;
            } else if (turnAngle != 0 && turn != 0) {
                if (Math.abs(average) >= (tick1 - tick2) && turn == 1){
                    pidDrive.setSetpoint(turnAngle);
                    turn++;
                    RobotLog.ii(TAG_TD, "start to avoid pole");
                } else if (Math.abs(average) >= tick1 + deltaTick && turn == 2){
                    pidDrive.setSetpoint(-2*turnAngle);
                    turn++;
                    RobotLog.ii(TAG_TD, "turn point");
                } else if (Math.abs(average) >= (tick1 + tick2 + 2*deltaTick) && turn == 3){
                    pidDrive.setSetpoint(turnAngle);
                    turn++;
                    RobotLog.ii(TAG_TD, "turn back");
                }
            }

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle(RotationAxis.CENTER));

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
                    setPower(basePowerR, basePowerL);
                else
                    setPower(basePowerL, basePowerR);
//                odometryLocalization.update();
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
//            odometryLocalization.update();
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

        int average2 = getEncoder();
        float inches = tickToInch(average);
        float inches2 = tickToInch(average2);
        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPID ending encoder value=" + average2 + " Before stopping=" +average);
        RobotLog.ii(TAG_TD, "target= "+ distanceInch + "ticks=" + tick + "Moved " + inches2 + "Before stopping=" + inches);
        //if (odometryEnabled)
//        odometryLocalization.showValues();
        RobotLog.ii(TAG_TD, "Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        op.idle();

        return inches2;
    }

    private double getAccelerationPower(double tick, double average, double delta, double maxPower) {
        double basePower = maxPower;
        //int thresholdTicks = Math.abs(tick / 4);
        int thresholdTicks = inchToTick(8);
        double base1 = thresholdTicks > 500 ? 500: 500; //thresholdTicks;
        double base2 = thresholdTicks > 1600 ? 1600 : 1600; //thresholdTicks;
        if (Math.abs(average) < thresholdTicks) {
            // ramp up for 1/4 of distance
            // minimum power is START_POWER
            // base is 1200
            //basePower = (average / 1200f) + Math.signum(tick) * POWER_START;
            basePower = (1.0*Math.abs(average) / base1) + POWER_START;
        } else if (Math.abs(delta) < thresholdTicks) {
            // ramp down for 3/4 of distance
            // minimum power is START_POWER
            // base is 3600
            //basePower = (delta / 3600f) + Math.signum(delta) * POWER_STOP;
            basePower = (1.0*Math.abs(delta) / base2) + POWER_STOP;
        }

        RobotLog.ii(TAG_TD, "accelerationPower: ticks= " + tick + " average= "+ average + " delta= "+delta + "basePower =" + basePower + "maxPower =" +maxPower);

        basePower = Range.clip(basePower, 0, maxPower);
        return basePower;
    }

    private double AngleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    private void turn(double power){
        driveLeftFront.setPower(-power);
        driveLeftBack.setPower(-power);
        driveRightFront.setPower(power);
        driveRightBack.setPower(power);
    }

//    /**
//     * Move the robot and also ensure that we are not over or under shooting
//     * @param distanceInch the distance to move in inches
//     * @param maxPower maximum motor power
//     * @param timeoutMillis maximum execution time for this function in milliseconds
//     * @param isBrake whether to break
//     * @param adjustLeftPower
//     * @param isAccelerate whether to ramp up and down. Use false for very short distance
//     * @param sensorDistance if specified (not null) then we detect the object in from of us
//     * @param actions
//     * @throws InterruptedException
//     */
//    public void moveToEncoderInchUsingPIDAndSensor(
//            float distanceInch,
//            float maxPower,
//            int timeoutMillis,
//            boolean isBrake,
//            boolean adjustLeftPower,
//            boolean isAccelerate,
//            DistanceSensor sensorDistance,
//            double startSensingDistance,
//            MoveAction... actions)
//            throws InterruptedException
//    {
//        int tick = inchToTick(distanceInch);
//        final float MOVE_TIMEOUT_FACTOR = 2f;
//        final int MOVE_STALL_TIME = 500;
//        float orgPower = maxPower;
//        float left_power_adj = 1f;
//        int targetEncoderValue = (int) (distanceInch * OcEncoder.COUNTS_PER_INCH);
//
//        maxPower = Math.abs(maxPower);
//        maxPower = Range.clip(maxPower, 0f, 1f);
//        //maxPower = 0.1f;
//        if (adjustLeftPower && orgPower > 0.6) {
//            left_power_adj = LEFT_POWER_ADJ;
//        }
//
//        if (timeoutMillis < 0) {
//            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
//        }
//
//        resetPosition();
//        // is RUN_TO_POSITION
//        setTargetPosition2(tick);
//        setMode2(RunMode.RUN_TO_POSITION);
//
//
//        // Set up parameters for driving in a straight line.
//        pidDrive.reset();
//        pidDrive.setSetpoint(0);
//        pidDrive.setOutputRange(0, orgPower);
//        pidDrive.setInputRange(-90, 90);
//        pidDrive.enable();
//
//        long startTimestamp = System.currentTimeMillis();
//        long timeStamp = startTimestamp;
//
//        Stalled stalled = new Stalled();
//        int average = getEncoder();
//        boolean forward = (distanceInch >= 0);
//        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor starting encoder value=" + average);
//        double stopDistance = 19.0;
//        double adjustDistance = stopDistance + 1.5;
//        boolean keepGoing = true;
//        while (op.opModeIsActive() &&
//                // is RUN_TO_POSITION
//                isBusy() &&
//                (timeStamp - startTimestamp < timeoutMillis) && keepGoing) {
//            if (odometryEnabled) {
////                if (odometryLocalization.getCurrentPosition() > targetEncoderValue) {
////                    RobotLog.ii(TAG_TD, "Reached odometry limit " + odometryLocalization.getCurrentPositionString());
////                    keepGoing = false;
////                }
//            }
//
//            average = getEncoder();
//            int delta = tick - average;
//            int deltaInch = tickToInch(Math.abs(delta));
//            RobotLog.ii(TAG_TD, "distance left deltaInch=" + deltaInch + " startSensingDistance=" + startSensingDistance);
//            if (Math.abs(delta) < 30)
//            {
//                // done
//                keepGoing = false;
//                RobotLog.ii(TAG_TD, "Reached distance limit");
//            } else if (isBrake && maxPower > 0.7f && deltaInch < 5) {
//                // slow down before stopping
//                maxPower *= 0.9;
//            }
//            if (sensorDistance != null) {
//                if (deltaInch <= startSensingDistance) {
//                    double distance = sensorDistance.getDistance(DistanceUnit.INCH);
//                    RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: sensor distance inches=" + distance);
//                    if (distance <= stopDistance) {
//                        if (keepGoing) {
//                            RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: reached sensor limit stop early");
//                            keepGoing = false;
//                        }
//                    } else if (distance <= adjustDistance && !keepGoing) {
//                        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor: did not reach reach sensor limit so keep going");
//                        keepGoing = true;
//                    }
//                }
//            }
//            if (!keepGoing) break;
//
//            // Use PID with imu input to drive in a straight line.
//            correction = pidDrive.performPID(getAngle());
//            RobotLog.ii(TAG_TD, "imu heading=" + lastAngles.firstAngle + " global heading=" + globalAngle + " correction=" + correction + " turn rotation=" + rotation);
//
//            //op.telemetry.addData("power", Float.toString(basePower));
//            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
//            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));
//
//            if (stalled.isStalled(average, timeStamp)) {
//                WaitLinear w = new WaitLinear(op);
//                stop2();
//                w.waitMillis(MOVE_STALL_TIME);
//                timeoutMillis += MOVE_STALL_TIME;
//                stalled.reset();
//            }
//            else {
//                double basePower = maxPower;
//                if (isAccelerate) {
//                    basePower = getAccelerationPower(tick, average, delta, maxPower);
//                }
//                double basePowerL = Math.signum(delta) * (basePower*left_power_adj - correction*basePower/maxPower);
//                //basePowerL = Range.clip(basePowerL, -maxPower, maxPower);
//                double basePowerR = Math.signum(delta) * (basePower + correction*basePower/maxPower);
//                //basePowerR = Range.clip(basePowerR, -maxPower, maxPower);
//                RobotLog.ii(TAG_TD, "basePowerL=" + basePowerL + " basePowerR=" + basePowerR);
//                if (forward)
//                    setPower(basePowerL, basePowerR);
//                else
//                    setPower(basePowerR, basePowerL);
//            }
//
//            if (actions != null) {
//                for (MoveAction action : actions) {
//                    if (action == null) {
//                        continue;
//                    }
//
//                    if (Math.abs(average) > Math.abs(action.distanceInTick())) {
//                        action.perform();
//                    }
//                }
//            }
//
//            //op.idle();
//            timeStamp = System.currentTimeMillis();
//        } // while
//
//        if (isBrake) {
//            //stop2();
//            setPower(0, 0);
//        }
//
//        if (Thread.currentThread().isInterrupted()) {
//            RobotLog.ee(TAG_TD, "Thread interrupted");
//        } else if (op.isStopRequested()) {
//            RobotLog.ee(TAG_TD, "Stop requested");
//        }
//
//        // is RUN_TO_POSITION
//        setMode2(RunMode.RUN_USING_ENCODER);
//
//        // stop actions
//        if (actions != null) {
//            for (MoveAction action : actions) {
//                if (action == null) {
//                    continue;
//                }
//
//                action.stop();
//                RobotLog.ii(TAG_TD, "Actions " + action + " stopped");
//            }
//            op.idle();
//        }
//
//        int average2 = getEncoder();
//        int inches = tickToInch(average);
//        RobotLog.ii(TAG_TD, "moveToEncoderInchUsingPIDAndSensor ending encoder value=" + average2 + " Before stopping=" +average);
//        RobotLog.ii(TAG_TD, "Moved " + inches + " inches" + tick +"ticks");
//        //if (odometryEnabled)
//        odometryLocalization.showValues();
//        RobotLog.ii(TAG_TD, "Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
//        //op.telemetry.addData("travel", inches);
//        op.idle();
//    }
}

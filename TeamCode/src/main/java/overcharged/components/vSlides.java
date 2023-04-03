package overcharged.components;

import static overcharged.config.RobotConstants.TAG_H;
import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;
import overcharged.util.PIDCalculator;

@Config
public class vSlides {
    ///Overcharged mecanum robot
    private RobotMecanum robot;
    //Slide motors
    public final OcMotorEx slideLeft;
    public final OcMotorEx slideMiddle;
    public final OcMotorEx slideRight;
    public OcSwitch switchSlideDown;
    public final List<OcSwitch> switchs = new ArrayList<>();

    //Slide level encoder values
    /*public static int level4 = 1970;
    public static int level3 = 1470;
    public static int level2 = 880;*/
    public static int level4 = 1390;//1949;
    public static int level3 = 990;//1410;
    public static int level2 = 590;
    public static int levelT = 300;
    public static int stack = 50;//100;
    public static int level1 = 85;//200;
    public static float factorR = 1;//0.957f;//0.963f;//1.015f;
    public static float factorL = 1f;
    //starting encoder reading
    public double start;
    ///Disable the slides if the switch says touched even at this encoder level of the slides
    public static int SLIDE_DISABLE_AT = 200;
    ///Slowdown the slide motor at this encoder level of the slides
    public static int SLIDE_SLOW_AT = 200;
    ///Speedup the slide motor at this encoder level of the slides
    public static int SLIDE_MOREPOWER_AT = 80;
    public static int max_level_value = level4+100; //lesser value of the two motors. Use tester to get this value

    ///Slide power constant
    float SLIDE_POWER_UP_PID = 1f;
    float SLIDE_POWER_DOWN_PID = -0.8f;
    public static float SLIDE_POWER_UP = 0.8f;
    public static float SLIDE_POWER_DOWN = -0.85f;
    public static float SLIDE_POWER_DOWN_OUT = -0.3f;
    private static float SLIDE_POWER_DOWN_MIN = -0.15f;
    static int SLIDE_POSITION_THRESHOLD = 5;

    public int currentPositionL = 0;
    public int currentPositionM = 0;
    public int currentPositionR = 0;

    /**
     * The state of the Slide
     */
    public enum State {
        UP,
        DOWN,
        STOP
    }

    ///Maintain the global slide state
    public State state = State.DOWN;
    ///Maintain the global slide previous state
    private State prev_state = State.DOWN;
    ///Initialize the PIDCalculator
    public static double p = 0.3;
    public static double i = 0;
    public static double d = 0;
    private PIDCalculator pidController = new PIDCalculator(p, i, d);
    //private PIDCalculator pidController = new PIDCalculator(0.2, 0, 0.1);
    ///maintain the previous state of the slides PID or not
    private boolean pidState = false;
    //private Arm arm;


    /**
     * Initialize the slide system
     */
    public vSlides(HardwareMap hardwareMap) {
        ///Initialize slide motors
        String missing = "";
        int numberMissing = 0;
        OcMotorEx slideL = null;
        try {
            slideL = new OcMotorEx(hardwareMap,
                    "slideL",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: slideL " + e.getMessage());
            missing = missing + ", slideL";
            numberMissing++;
        }
        this.slideLeft = slideL;

        OcMotorEx slideM = null;
        try {
            slideM = new OcMotorEx(hardwareMap,
                    "slideM",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: slideM " + e.getMessage());
            missing = missing + ", slideM";
            numberMissing++;
        }
        this.slideMiddle = slideM;

        OcMotorEx slideR = null;
        try {
            slideR = new OcMotorEx(hardwareMap,
                    "slideR",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: slideR " + e.getMessage());
            missing = missing + ", slideR";
            numberMissing++;
        }
        this.slideRight = slideR;

        OcSwitch lswitch = null;
        try {
            lswitch = new OcSwitch(hardwareMap,"limitswitch", true);
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            switchs.add(lswitch);
            RobotLog.ii(TAG_H, "limitSwitch(isNull)? " + isSwitchNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            missing = missing + ", limitswitch";
            numberMissing++;
            boolean isSwitchNull= switchSlideDown==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isSwitchNull);
        }
        this.switchSlideDown = lswitch;
        initialize(this.slideLeft);
        initialize(this.slideMiddle);
        initialize(this.slideRight);
        resetSlidePosition();
        RobotLog.ii(TAG_SL, "Initialized the Slide component numberMissing=" + numberMissing + " missing=" + missing);
        RobotLog.ii(TAG_SL, "Initialized the Slide component slideLeft=" + slideLeft.getCurrentPosition() + " slideMiddle=" + slideMiddle.getCurrentPosition() + " slideRight=" + slideRight.getCurrentPosition());
    }

    public boolean slideReachedBottom() {
        if (switchSlideDown.isDisabled()) return slideLeft.getCurrentPosition() <= start;
        return isSlideSwitchPressed();
    }

    public void resetSlidePosition(){
        currentPositionL = 0;
        currentPositionM = 0;
        currentPositionR = 0;
        start = slideLeft.getCurrentPosition();
    }

    public double getCurrentPosition(){
        return slideMiddle.getCurrentPosition();
    }

    public boolean isSlideSwitchPressed() {
        return switchSlideDown.isTouch();
    }

    private void initialize(OcMotorEx motor) {
        reset(motor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void reset(OcMotorEx motor) {
        motor.setPower(0f);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.resetPosition();
    }

    /**
     * Check if the slide reached the max upper limit using encoders
     *
     * @return
     */
    private boolean reachedMaxUpperLimit() {
        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideMiddle.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
        RobotLog.ii(TAG_SL, "reachedMaxUpperLimit() positionL=" + positionL + " positionM=" + positionM + " positionR=" + positionR + " max_level_value=" + max_level_value);
        if (positionL > max_level_value || positionR > max_level_value) {
            RobotLog.ii(TAG_SL, "Reached max level positionL=" + positionL + " positionM=" + positionM + " positionR=" + positionR + " max_level_value=" + max_level_value);
            return true;
        }
        return false;
    }

    ///	 Get the encoder value for the requested level, to move to
    public int getDistanceL(int pos)
    {
        int positionL = slideLeft.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionL;
        return distance;
    }

    public int getDistanceM(int pos)
    {
        int positionM = slideMiddle.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionM;
        return distance;
    }

    ///	 Get the encoder value for the requested level, to move to
    public int getDistanceR(int pos)
    {
        int positionR = slideRight.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionR;
        return distance;
    }

    /**
     * Move up the slides to the top most level
     */
    public void moveTo4()
    {
        moveSlidesTo(level4);
    }
    public void moveTo3()
    {
        moveSlidesTo(level3);
    }
    public void moveTo2()
    {
        moveSlidesTo(level2);
    }
    public void moveToT()
    {
        moveSlidesTo(levelT);
    }
    public void moveTo1()
    {
        moveSlidesTo(level1);
    }
    public void moveTo(int level)
    {
        moveSlidesTo(level);
    }


    /*public void moveToTop(boolean auto)
    {
        if(auto){
            moveSlidesTo( level3 + 50);
        }
    }*/



    /**
     * Move up the slides to the mid level
     */
    /*public void moveToMid()
    {
        moveSlidesTo( level2);
    }*/

    /**
     * Move the slides to the last bottom level
     */
    public void moveToBottom()
    {
        //moveSlidesTo( 0);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        move(SLIDE_POWER_DOWN);
    }

    /**
     * @return the power from the PID calculations
     */
    public double getPidPower(int position, int level_value) {
        int wantedPos = position + level_value;
        int error = wantedPos - position;
        return pidController.getPID(error);
    }

    ///Is the slide at the position where we want to check for switch is disabled
    private boolean isAtDisablePosition()
    {
        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideMiddle.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
        if (positionL > SLIDE_DISABLE_AT || positionM > SLIDE_DISABLE_AT || positionR > SLIDE_DISABLE_AT) return true;
        return false;
    }

    ///Is the slide at the position where we want to slow down the slide motors
    private boolean isAtLowPosition()
    {
        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideMiddle.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
        RobotLog.ii(TAG_SL, "isAtLowPosition positionL=" + positionL + " positionM=" + positionM + " positionR=" + positionR + " SLIDE_SLOW_AT=" + SLIDE_SLOW_AT);
        if (positionL < SLIDE_SLOW_AT || positionM < SLIDE_SLOW_AT || positionR < SLIDE_SLOW_AT) return true;
        return false;
    }

    ///Is the slide almost at the bottom position where we want to more power to tighten and bring down the slide
    private boolean isAlmostBottomPosition()
    {
        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideMiddle.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
        RobotLog.ii(TAG_SL, "isAlmostBottomPosition positionL=" + positionL + " positionM=" + positionM + " positionR=" + positionR + " SLIDE_MOREPOWER_AT=" + SLIDE_MOREPOWER_AT);
        if (positionL < SLIDE_MOREPOWER_AT || positionM < SLIDE_MOREPOWER_AT || positionR < SLIDE_MOREPOWER_AT) return true;
        return false;
    }

    /**
     * Keep the slide system at the current level at a specified power using PID
     */
    public void keep() {
        if (pidState) return;
        state = State.STOP;
        if (prev_state == state) return;
        prev_state = state;
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideLeft.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
        RobotLog.ii(TAG_SL, "Keep the slide at current level positionL=" + positionL + " positionM=" + positionM + " positionR=" + positionR);
        float powerL = (float)(pidController.getPID(positionL));
        float powerM = (float)(pidController.getPID(positionM));
        float powerR = (float)(pidController.getPID(positionR));
        slideLeft.setTargetPosition(positionL);
        slideMiddle.setTargetPosition(positionM);
        slideRight.setTargetPosition(positionR);
        currentPositionL = positionL;
        currentPositionM = positionM;
        currentPositionR = positionR;
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMiddle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(powerL);
        slideMiddle.setPower(powerM);
        slideRight.setPower(powerR);
    }

    private boolean isInRange(int x, int y)
    {
        if (Math.abs(x-y) < SLIDE_POSITION_THRESHOLD) {
            return true;
        }
        return false;
    }

    /**
     * move the slide system up/down to the specified level at a calculated power using PID
     * @param pos encoder value of the level we want to reach
     */
    private void moveSlidesTo(int pos) {
        pidState = true;
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int positionL = slideLeft.getCurrentPosition();
        int positionM = slideMiddle.getCurrentPosition();
        int positionR = slideRight.getCurrentPosition();
//        if (isInRange(positionL, currentPositionL) ||
//                isInRange(positionR, currentPositionR)) {
//            positionL = currentPositionL;
//            positionR = currentPositionR;
//        }
        int distanceL = getDistanceL(pos);
        int distanceM = getDistanceM(pos);
        int distanceR = getDistanceR(pos);
        if (Math.signum(distanceL) > 0) {
            state = State.UP;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level up to " + pos);
            if (reachedMaxUpperLimit()) {
                RobotLog.ii(TAG_SL, "Slide reached max level so quitting");
                setPower(0f);
                return;
            }
            RobotLog.ii(TAG_SL, "vSlides: Slide up positionL=" + positionL + " positionR=" + positionR + " SLIDE_DISABLE_AT=" + SLIDE_DISABLE_AT);
            if (isAtDisablePosition() && (switchSlideDown != null) &&
                    !switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                /// If the slide down switch is still showing as on/isTouch() then there is a problem with the switch (not connected or broken)
                /// So disable it to allow slide down functionality
                RobotLog.ii(TAG_SL, "Slide up switchSlideDown.disable()");
                switchSlideDown.disable();
                //if (robot != null) robot.ledRedBlink();
            }
        } else {
            state = State.DOWN;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level down to " + pos);
            if (!switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                reset(slideLeft);
                reset(slideMiddle);
                reset(slideRight);
                resetSlidePosition();
                RobotLog.ii(TAG_SL, "Slide switch touched");
                return;
            }
        }
        int gotoPositionL = positionL + distanceL;
        if (gotoPositionL < 0) gotoPositionL = 0;
        int gotoPositionM = positionM + distanceM;
        if (gotoPositionM < 0) gotoPositionM = 0;
        int gotoPositionR = positionR + distanceR;
        if (gotoPositionR < 0) gotoPositionR = 0;
        float power = (float)(getPower(slideMiddle, pos));
        RobotLog.ii(TAG_SL, "vSlides: Move the slide to position='" + pos + "' distanceL='" + distanceL + "' distanceR='" + distanceR  + "' gotoPositionL='" + gotoPositionL  + "' gotoPositionR='" + gotoPositionR  + "' power='" + power + "'");
        slideLeft.setTargetPosition((int)(factorL*gotoPositionL));
        slideMiddle.setTargetPosition((int)(gotoPositionM));
        slideRight.setTargetPosition((int)(factorR*gotoPositionR));
        currentPositionL = gotoPositionL;
        currentPositionM = gotoPositionM;
        currentPositionR = gotoPositionR;
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMiddle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(power);
        slideMiddle.setPower(power);
        slideRight.setPower(power);
        RobotLog.ii(TAG_SL, "powerL " + slideLeft.getPower() + " powerM " + slideMiddle.getPower() + " powerR " + slideRight.getPower());
    }

    public double getPower(OcMotorEx slide, double gotoPosition) {
        double currentPosition = slide.getCurrentPosition();
        double distance = gotoPosition - currentPosition;
        float power = Math.signum(distance) < 0 ? SLIDE_POWER_DOWN_PID: SLIDE_POWER_UP_PID;
        RobotLog.ii(TAG_SL, "getPower sign=" + Math.signum(distance) + " gotoPosition='" + gotoPosition +  "' currentPosition='" + currentPosition + "' power='" + power + "'");
        return power;
//        double retpower = Range.clip( (distance/max_level_value) * power, -1.0, 1.0);
//        retpower = Math.abs(retpower) < 0.7 ? Math.signum(retpower) * 0.7 : retpower;
//        RobotLog.ii(TAG_SL, "getPower sign=" + Math.signum(distance) + " gotoPosition='" + gotoPosition +  "' currentPosition='" + currentPosition + "' power='" + power +  "' retpower='" + retpower + "'");
//        return retpower;
    }

    /**
     * turn the slides system at a specified power and direction until it reaches the predetermined maximum
     *
     * @param up when true moves the slide up. false moves the slide down
     * @param powerFactor the power factor, using this value make it slower or full speed
     */
    private void moveSlides(boolean up, float powerFactor) {
        pidState = false;
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMiddle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastPowerFactor = powerFactor;
        if (up) {
            RobotLog.ii(TAG_SL, "Move the slide up powerFactor=" + powerFactor);
            state = State.UP;
            if (reachedMaxUpperLimit()) {
                RobotLog.ii(TAG_SL, "Slide reached max level so quitting");
                setPower(0f);
                return;
            }
            setPower(powerFactor * SLIDE_POWER_UP);
            if (prev_state != state) prev_state = state;
            if (isAtDisablePosition() && (switchSlideDown != null) &&
                    !switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                /// If the slide down switch is still showing as on/isTouch() then there is a problem with the switch (not connected or broken)
                /// So disable it to allow slide down functionality
                RobotLog.ii(TAG_SL, "Slide up switchSlideDown.disable()");
                switchSlideDown.disable();
                //if (robot != null) robot.ledRedBlink();
            }
        } else {
            RobotLog.ii(TAG_SL, "Move the slide down powerFactor=" + powerFactor);
            state = State.DOWN;
            if (prev_state != state) prev_state = state;
            if (!slideReachedBottom()) {
                setPower(getSlideDownPower(powerFactor));
            } else {
                RobotLog.ii(TAG_SL, "switchSlideDown is touched and slide left position=" + slideLeft.getCurrentPosition());
                reset(slideLeft);
                reset(slideMiddle);
                reset(slideRight);
                resetSlidePosition();
            }
        }
    }

    private float getSlideDownPower(float powerFactor) {
        float power = powerFactor * SLIDE_POWER_DOWN;
//        if (isAlmostBottomPosition() && !(getFourBarStatus() == FourBar.FourBarState.OUT))
//            power = SLIDE_POWER_DOWN;
//        else if (isAtLowPosition() || getFourBarStatus() == FourBar.FourBarState.OUT)
//            power = (powerFactor > .5f) ? (powerFactor * SLIDE_POWER_DOWN_OUT) : SLIDE_POWER_DOWN_MIN;
        /*if (arm != null && arm.armOut)
            power = (powerFactor > .5f) ? (powerFactor * SLIDE_POWER_DOWN_OUT) : SLIDE_POWER_DOWN_MIN;*/
        RobotLog.ii(TAG_SL, "getSlideDownPower=" + power);
        return power;
    }

    /**
     * Move the slides up/down
     * if y > 0 the slide moves up
     * if y < 0 the slide moves down
     */
    public void move(float y)
    {
        RobotLog.ii(TAG_SL, "move slide y " + y);
        if (y == 0) return;
        boolean up = (y > 0) ? true : false;
        if (up) {
            moveSlides(true, y);
        } else {
            moveSlides(false,Math.abs(y));
        }
    }

    /**
     * Save the last power used so that we can use it to perform some automation.
     * Use it to keep the slides stationary up/down
     */
    private float lastPowerFactor = 0f;

    /**
     * stop the slide motors
     */
    public void stop()
    {
        if (pidState) return;
        if (prev_state != State.STOP) {
            RobotLog.ii(TAG_SL, "Stop the Slide component");
            setPower(0f);
            prev_state = State.STOP;
        }
        state = State.STOP;
        lastPowerFactor = 0f;
    }

    /**
     * stop the slide motors
     */
    public void forcestop()
    {
        //if (prev_state != State.STOP) {
            RobotLog.ii(TAG_SL, "Force stop the Slide component");
            setPower(0f);
            prev_state = State.STOP;
        //}
        state = State.STOP;
        lastPowerFactor = 0f;
        RobotLog.ii(TAG_SL, "powerL " + slideLeft.getPower() + " powerM " + slideMiddle.getPower() + " powerR " + slideRight.getPower());
    }

    /**
     * Set the power on both the motors. Also accommodate and report for any motor failures.
     * @param power power to set
     */
    public void setPower(float power)
    {
        int cnt = 1;
        //try {
        if (slideLeft != null) {
            RobotLog.ii(TAG_SL, "Set slide left motor power to " + power);
            slideLeft.setPower(power);
            if (power == 0f) {
                slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorL");
        }

        cnt = 2;
        //try {
        if (slideMiddle != null) {
            RobotLog.ii(TAG_SL, "Set slide middle motor power to " + power);
            slideMiddle.setPower(power);
            if (power == 0f) {
                slideMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorM");
        }

        cnt = 3;
        if (slideRight != null) {
            RobotLog.ii(TAG_SL, "Set slide right motor power to " + power);
            slideRight.setPower(power);
            if (power == 0f) {
                slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            RobotLog.ii(TAG_SL, "Not setting power for motorR");
        }
        /*} catch (Exception e) {
            if (cnt == 1) {
                RobotLog.ii(TAG_SL, "Issue with left motor so turn off the slides: " + e.getStackTrace());
                motorL = null;
            }
            if (cnt == 2) {
                RobotLog.ii(TAG_SL, "Issue with right motor so turn off the slides: " + e.getStackTrace());
                motorR = null;
            }
        } */
    }

    public void stopMotors(){
        slideLeft.setPower(0f);
        slideMiddle.setPower(0f);
        slideRight.setPower(0f);
    }
}
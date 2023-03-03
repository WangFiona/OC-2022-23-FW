package overcharged.components;

import static overcharged.config.RobotConstants.TAG_SL;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.util.PIDCalculator;

public class Turret {
    public OcMotorEx turret;
    public static float TURRET_POWER = 0.8f;

    public int currentPos = 0;
    public double start;
    public double maxPos = 1500;

    public static double p = 1.8;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;

    public static double p2 = 15;
    public static double i2 = 0;
    public static double d2 = 0.4;
    public static double f2 = 0;
    private PIDCalculator pidController = new PIDCalculator(p, i, d);
    private boolean pidState = false;
    private boolean negative = true;
    private double angleToEncoder = 1705/180f;
    private double encoderToAngle = 180/1705f;

    public int currentPosition = 0;

    public Turret(HardwareMap hardwareMap){
        turret = new OcMotorEx(hardwareMap, "turret", DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        turret.resetPosition();
        turret.setPower(0f);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //PIDFCoefficients pidNew = new PIDFCoefficients(p, i, d, f);
        turret.setTargetPositionPIDFCoefficients(p, i, d, f);
        //turret.setVelocityPIDFCoefficients(p2, i2, d2, f2);
    }

    public void resetTurretPosition(){
        currentPos = 0;
        start = turret.getCurrentPosition();
    }

    public void isNegative(boolean negat){
        negative = negat;
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

    public void moveEncoderTo(int pos, float power){
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);//*factor);
    }

    public void moveTo2(double angle, float power, boolean clockwise){
        double current = getCurrentAngle();
        double difference = angle - current;
        RobotLog.ii(TAG_SL, "turret angle " + angle + " turret current " + current + " turret dff " + difference);
        if(!clockwise){
            while (difference > 0){
                difference -= 360;
            }
            if(difference < -360)
                difference += 360;
        } else {
            while (difference < 0){
                difference += 360;
            }
            if(difference > 360)
                difference -= 360;
        }
        double goTo = turret.getCurrentPosition()+(difference*angleToEncoder);
        moveEncoderTo((int)(goTo), power);
    }

    public void moveTo(double pos, float power){
        double encoder = 0;
        if (pos < 0) {
            encoder = (pos / 180) * 1705;//1730;
            RobotLog.ii(TAG_SL, "turret pos<0");
        }
        if (pos > 0) {
            encoder = (pos / 180) * 1705;//1730;
            RobotLog.ii(TAG_SL, "turret pos>0");
        }
        RobotLog.ii(TAG_SL, "turret encoder " + encoder + " turret pos " + pos);
        RobotLog.ii(TAG_SL, "turret pidf " + turret.getPIDCoefficients());//p + " " +  i + " " + d + " " + f);
        moveEncoderTo((int)(encoder), power);
    }

    public double getCurrentAngle() {
        double pos = turret.getCurrentPosition();
        double angle = pos*encoderToAngle;
        RobotLog.ii(TAG_SL, "turret encoderpos " + pos + " turret angle " + angle + " turret encoderToAngle " + encoderToAngle);
        /*if(negative) {
            angle = pos*encoderToAngle;
            //RobotLog.ii(TAG_SL, "turret angle " + angle);
        } else {
            if (pos > 0) {
                angle = pos*encoderToAngle;
            }
            if (pos < 0) {
                angle = 360 + pos*encoderToAngle;
            }
        }*/
        return angle;
    }

    public double getCurrentPosition(){
        return turret.getCurrentPosition();
    }

    ///	 Get the encoder value for the requested level, to move to
    public int getDistance(int pos)
    {
        int positionL = turret.getCurrentPosition();
        //calculate the distance to travel using the left side slide motor
        int distance = pos - positionL;
        return distance;
    }

    /*private void moveTurretTo(int pos) {
        pidState = true;
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int position = turret.getCurrentPosition();
//        if (isInRange(positionL, currentPositionL) ||
//                isInRange(positionR, currentPositionR)) {
//            positionL = currentPositionL;
//            positionR = currentPositionR;
//        }
        int distance = getDistance(pos);
        if (Math.signum(distance) > 0) {
            state = vSlides.State.UP;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level up to " + pos);
            if (reachedMaxUpperLimit()) {
                RobotLog.ii(TAG_SL, "Slide reached max level so quitting");
                setPower(0f);
                return;
            }
            RobotLog.ii(TAG_SL, "Slide up positionL=" + positionL + " positionR=" + positionR + " SLIDE_DISABLE_AT=" + SLIDE_DISABLE_AT);
            if (isAtDisablePosition() && (switchSlideDown != null) &&
                    !switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                /// If the slide down switch is still showing as on/isTouch() then there is a problem with the switch (not connected or broken)
                /// So disable it to allow slide down functionality
                RobotLog.ii(TAG_SL, "Slide up switchSlideDown.disable()");
                switchSlideDown.disable();
                //if (robot != null) robot.ledRedBlink();
            }
        } else {
            state = vSlides.State.DOWN;
            if (prev_state != state) prev_state = state;
            RobotLog.ii(TAG_SL, "Move the slide level down to " + pos);
            if (!switchSlideDown.isDisabled() && isSlideSwitchPressed()) {
                reset(turret);
                resetSlidePosition();
                RobotLog.ii(TAG_SL, "Slide switch touched");
                return;
            }
        }
        int gotoPosition = position + distance;
        if (gotoPosition < 0) gotoPosition = 0;
        double power = getPower(turret, pos);
        RobotLog.ii(TAG_SL, "Move the slide to position='" + pos + "' distance='" + distance  + "' gotoPositionL='" + gotoPositionL  + "' gotoPositionR='" + gotoPositionR  + "' power='" + power + "'");
        turret.setTargetPosition(gotoPosition);
        currentPosition = gotoPosition;
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }*/

    public void on(){
        turret.setPower(0.485f);
    }

    public void mid(){
        turret.setPower(0.44f);
    }

    public void setPower(float pwr){
        turret.setPower(pwr);
    }

    public void max(){
        turret.setPower(1);
    }

    public void off(){
        turret.setPower(0);
    }
}

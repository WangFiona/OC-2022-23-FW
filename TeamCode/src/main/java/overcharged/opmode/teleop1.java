package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;
import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import overcharged.components.Button;
import overcharged.components.MecanumDrive;
import overcharged.components.RobotMecanum;
import overcharged.components.hSlides;

/**
 * Overcharged Team #12599
 * Teleop for Mecanum robot blue alliance
 */
@Config
@TeleOp(name="teleop1", group="Game")
public class teleop1 extends OpMode {
    ///Overcharged Robot object
    RobotMecanum robot;
    MecanumDrive drive;
    ///Slow mode power factor
    private final static float SLOW_POWER_MULT = 0.65f;
    double slowPower = 1;
    boolean isSlow = false;
    private boolean retractdeposit = false;

    long startTime;

    ClawState clawState = ClawState.OPEN;
    SlideLocation slideLocation = SlideLocation.BOTTOM;
    boolean slideGoBottom = false;
    boolean slideManualDown = false;
    boolean slideOn = false;
    boolean turretOn = true;
    double turretPos;
    double slidePos;
    boolean slideDown = false;
    double turretPower = 0.9;
    boolean goZero = false;
    boolean zeroDown = false;
    int counter = 0;
    boolean justAdjusted = false;
    //float slideInit = 190f;
    float hSlidePos = hSlides.START;
    boolean slightDown = false;
    int downFlag = 0;
    int levelFlag = 0;
    int level = 0;
    ButtonState buttonState = ButtonState.NO2;
    int turretAdjust = 0;
    boolean clawExtending = false;
    int clawFlag = 0;
    boolean right = false;
    boolean turretSlow = false;
    boolean turret90 = false;
    boolean turretN90 = false;
    long hSlidesTime;
    boolean hPosChanged = false;
    double stepSize = 2;
    SlideMode slideMode = SlideMode.NORMAL;
    double previousPower = 0.5f;
    boolean powerSlow = false;

    boolean goToZero = false;

    public enum ClawState{
        OPEN,
        GRAB;
    }

    public enum ButtonState{
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }

    public enum SlideLocation {
        BOTTOM,
        L1,
        LT,
        L2,
        L3,
        L4;
    }

    public enum SlideMode {
        NORMAL,
        CONE;
    }

    @Override
    public void init() {
        try {
            RobotLog.ii(TAG_T, "Teleop init start");
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new RobotMecanum(this, false, false);
            RobotLog.ii(TAG_T, "Teleop init RobotMecanum initialized");
            startTime = System.currentTimeMillis();
            robot.setBulkReadManual();
            RobotLog.ii(TAG_T, "Enabled BulkCachingMode.Manual");
            retractdeposit = false;
            RobotLog.ii(TAG_T, "Teleop init slide position reset");
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        } catch (Exception e) {
            RobotLog.ee(TAG_T, "Teleop init failed: " + e.getMessage());
            telemetry.addData("Init Failed", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        // must clear bulk cache for BulkCachingMode.MANUAL
        robot.clearBulkCache();

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*slowPower;
        double backLeftPower = ((y - x + rx) / denominator)*slowPower;
        double frontRightPower = ((y - x - rx) / denominator)*slowPower;
        double backRightPower = ((y + x - rx) / denominator)*slowPower;

        robot.driveLeftFront.setPower(frontLeftPower);
        robot.driveLeftBack.setPower(backLeftPower);
        robot.driveRightFront.setPower(frontRightPower);
        robot.driveRightBack.setPower(backRightPower);

        //check slow mode
        if (gamepad1.a && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = !isSlow;
        }

        // turret slow mode
        if(gamepad2.right_trigger > 0.9){
            turretPower = 0.15;
            stepSize = 0.5;
        } else {
            turretPower = 0.9;
            stepSize = 2;
        }

        /**
         * Turn on slow mode and blink blue indicatior for the drivers
         */
        if (isSlow) {
            slowPower = 0.5f;
            y *= SLOW_POWER_MULT;
            x *= SLOW_POWER_MULT;
            rx *= SLOW_POWER_MULT;
            //robot.ledBlueBlink();
        } else {
            slowPower=1;
            //robot.ledBlueOn(false);
        }

        /*if(gamepad1.y && Button.BTN_STRAIGHTEN.canPress(timestamp)){
            if(Math.abs(drive.getAngle(RotationAxis.CENTER))<45) {
                try {
                    drive.turnUsingPID(0, 0.5f, RotationAxis.CENTER);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else if(Math.abs(drive.getAngle(RotationAxis.CENTER))>135){
                try {
                    drive.turnUsingPID(180, 0.5f, RotationAxis.CENTER);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else{
                try {
                    drive.turnUsingPID(90, 0.5f, RotationAxis.CENTER);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }*/

        double hS = gamepad2.left_stick_y;

        if(Math.abs(hS) > 0.3){
            if(timestamp - hSlidesTime > 10){
                hPosChanged = true;
                hSlidePos += stepSize*Math.signum(hS);
                hSlidesTime = timestamp;
            }
        }

        if(hSlidePos > hSlides.MAX)
            hSlidePos = hSlides.MAX;
        if (hSlidePos < hSlides.MIN)
            hSlidePos = hSlides.MIN;

        if(hPosChanged) {
            robot.hSlides.setPosition(hSlidePos);
            hPosChanged = false;
        }

        if((robot.turret.getCurrentPosition() < 0 && robot.turret.getCurrentPosition() < -1810)
                || (robot.turret.getCurrentPosition() > 0 && robot.turret.getCurrentPosition() > 1560))
            turretOn = false;
        else
            turretOn = true;

        if(gamepad1.left_trigger > 0.9 && Button.TURRET_RESET.canPress(timestamp))
            robot.turret.reset(robot.turret.turret);

        if(gamepad1.dpad_up && Button.SLIDE_RESET.canPress(timestamp)){
            slideLocation = SlideLocation.BOTTOM;
            robot.clawGrab();
            slideManualDown = true;
        }

        if(slideManualDown)
            manualDown();

        //Manual controls for turret
        double tx = -gamepad2.right_stick_x;
        if(turretOn && Math.abs(tx) > 0.1) {
            if(slideLocation == SlideLocation.BOTTOM || slideLocation == SlideLocation.L1) {
                robot.vSlides.moveTo2();
                slideLocation = SlideLocation.L2;
            }
            if(robot.vSlides.getCurrentPosition() > 400) {
                if (tx > 0 && robot.turret.getCurrentPosition() < 1560) {
                    robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.turret.setPower(turretPower);
                } else if(robot.turret.getCurrentPosition() > -1810) {
                    robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.turret.setPower(-turretPower);
                }
            }
        } else{
            if(!goZero)
                robot.turret.setPower(0);
        }

        if(gamepad2.a && Button.BTN_TURRET.canPress(timestamp)){
            robot.hSlides.setPosition(hSlides.MAX);
            hSlidePos = hSlides.MAX;
            robot.turret.setPower(0);
            robot.clawGrab();
            turretAdjust = 0;
            goZero = true;
            telemetry.addLine("return to zero");
            telemetry.update();
            robot.turret.moveTo(turretAdjust, turretPower);
            zeroDown = true;
        }

        if(gamepad2.x && Button.TURRET_RIGHT.canPress(timestamp) && slideLocation != SlideLocation.BOTTOM && slideLocation != SlideLocation.L1){
            robot.turret.setPower(0);
            turretAdjust = 87;
            robot.turret.moveTo(turretAdjust, turretPower);
            goZero = true;
            turret90 = true;
            /*
            robot.turret.setPower(0);
            clawExtending = true;
            robot.hSlides.setPosition(125f);
            turretAdjust = 87;//-450;
            goZero = true;
            clawFlag = 0;
            right = true;
            */
        }

        if(gamepad2.b && Button.TURRET_LEFT.canPress(timestamp) && slideLocation != SlideLocation.BOTTOM && slideLocation != SlideLocation.L1){
            robot.turret.setPower(0);
            turretAdjust = -87;
            robot.turret.moveTo(turretAdjust, turretPower);
            goZero = true;
            turretN90 = true;
            /*
            robot.turret.setPower(0);
            clawExtending = true;
            robot.hSlides.setPosition(123f);
            turretAdjust = -87;//-450;
            goZero = true;
            clawFlag = 0; */
        }

        if(turret90 && robot.turret.getCurrentAngle() < 89 && robot.turret.getCurrentAngle() > 85) {
            turret90 = false;
            goZero = false;
        }

        if(turretN90 && robot.turret.getCurrentAngle() > -89 && robot.turret.getCurrentAngle() < -85) {
            turretN90 = false;
            goZero = false;
        }

        if(clawExtending && clawFlag > 40){
            clawFlag = 0;
            robot.turret.moveTo(turretAdjust, turretPower);
            zeroDown = true;
            justAdjusted = true;
            clawExtending = false;
        } else{
            clawFlag++;
        }

        if(robot.turret.getCurrentAngle() > 50 && right){
            robot.hSlides.setPosition(hSlides.PRESET1);
            right = false;
        }

        if(zeroDown && (Math.abs(Math.abs(robot.turret.getCurrentAngle())-Math.abs(turretAdjust)) < 2)){
            robot.turret.moveTo(turretAdjust, turretPower);
            slideLocation = SlideLocation.BOTTOM;
            goZero = false;
            zeroDown = false;
            robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if((turretPos < 1515 && turretPos > 970) || (turretPos < -875 && turretPos > -1820)){
            slideDown = false;
        }
        else{
            slideDown = true;
        }

        if(robot.vSlides.slideLeft.getCurrentPosition() > robot.vSlides.level3+30){
            powerSlow = true;
            //previousPower = slowPower;
            slowPower = 0.3f;
        } else if(robot.vSlides.slideLeft.getCurrentPosition() > robot.vSlides.level3-80){
            powerSlow = true;
            previousPower = slowPower;
            slowPower = 0.4f;
        } else if(robot.vSlides.slideLeft.getCurrentPosition() > robot.vSlides.level2-50) {
            powerSlow = true;
            previousPower = slowPower;
            slowPower = 0.7f;
        } else if(powerSlow){
            powerSlow = false;
            slowPower = previousPower;
        }

        //robot.vSlides.slideLeft.getCurrentPosition() < robot.vSlides.level1-30
        if((slideLocation == SlideLocation.BOTTOM || slideLocation == SlideLocation.L1) && Math.abs(robot.turret.getCurrentPosition()) < 200) {
            robot.turret.moveTo(0, turretPower);
            goToZero = true;
        }else
            goToZero = false;

        turretPos = robot.turret.getCurrentPosition();
        telemetry.addData("goToZero", goToZero);
        telemetry.addData("slowPower", slowPower);
        telemetry.addData("sensorF", robot.sensorF.getDistance(DistanceUnit.CM));
        telemetry.addData("sensorL", robot.sensorL.getRawLightDetected());
        telemetry.addData("sensorR", robot.sensorR.getRawLightDetected());
        telemetry.addData("turret encoder ", turretPos);
        telemetry.addData("slideL encoder ", robot.vSlides.slideLeft.getCurrentPosition());
        telemetry.addData("slideR encoder ", robot.vSlides.slideRight.getCurrentPosition());
        telemetry.addData("limit switch ", robot.vSlides.isSlideSwitchPressed());
        telemetry.addData("hSlide getPos ", robot.hSlides.getPos());
        telemetry.addData("hSlidePos ", hSlidePos);
        telemetry.addData("hS ", hS);
        telemetry.addData("slide Location ", level);
        telemetry.addData("driveSlow ", isSlow);
        telemetry.addData("turretSlow ", turretSlow);
        telemetry.addData("L", robot.driveLeftBack.getCurrentPosition());
        telemetry.addData("M", robot.driveRightBack.getCurrentPosition());
        telemetry.addData("R", robot.driveRightFront.getCurrentPosition());
        telemetry.addData("tx", tx);
        telemetry.update();

        /*//Manual controls for slides
        if(gamepad1.b && Button.BTN_SLIDEON.canPress(timestamp)){
            //telemetry.addLine("button b pressed");
            //telemetry.update();
            if(slideOn){
                //telemetry.addLine("button b pressed 0");
                //telemetry.update();
                robot.vSlides.setPower(0);
                slideOn = false;
            } else if(!slideOn){
                //telemetry.addLine("button b pressed 0.7");
                //telemetry.update();
                robot.vSlides.setPower(0.7f);
                slideOn = true;
            }
        }

        if(gamepad1.a && Button.BTN_SLIDEDOWN.canPress(timestamp)){
            //telemetry.addLine("button a pressed");
            //telemetry.update();
            if(slideDown){
                //telemetry.addLine("button a pressed 0");
                //telemetry.update();
                robot.vSlides.setPower(0);
                slideOn = false;
            } else if(!slideDown){
                //telemetry.addLine("button b pressed -0.7");
                //telemetry.update();
                robot.vSlides.setPower(-0.7f);
                slideOn = true;
            }
        }*/

        if(gamepad2.y && Button.RESET_H.canPress(timestamp)){
            if(slideMode == SlideMode.NORMAL) {
                hSlidePos = hSlides.INIT;
                robot.hSlides.setPosition(hSlidePos);
            } else{
                hSlidePos = 150;
                robot.hSlides.setPosition(150);
            }
        }

        //Controls for claw
        if((gamepad1.right_bumper || gamepad2.right_bumper) && Button.BTN_OPEN.canPress(timestamp)){
            if(clawState == ClawState.GRAB && !slideGoBottom){
                robot.clawOpen();
                clawState = ClawState.OPEN;
            }else if(clawState == ClawState.OPEN){
                robot.clawGrab();
                if(slideLocation == SlideLocation.BOTTOM && slideMode == SlideMode.NORMAL)
                    robot.vSlides.moveTo1();
                clawState = ClawState.GRAB;
            }
        }

        if(gamepad1.left_bumper && slideLocation != SlideLocation.BOTTOM){
            if(buttonState == ButtonState.NO2) {
                buttonState = ButtonState.PRESSED;
            }
            else {
                buttonState = ButtonState.PRESSING;
            }
        } else if(slideLocation != SlideLocation.BOTTOM){
            if(buttonState == ButtonState.PRESSING)
                buttonState = ButtonState.NO;
            else
                buttonState = ButtonState.NO2;
        }

        if(slideLocation != SlideLocation.BOTTOM) {
            if (slideLocation == SlideLocation.L1)
                level = robot.vSlides.level1;
            else if (slideLocation == SlideLocation.L2)
                level = robot.vSlides.level2;
            else if (slideLocation == SlideLocation.L3)
                level = robot.vSlides.level3;
            else if (slideLocation == SlideLocation.L4)
                level = robot.vSlides.level4;
        }

        if(buttonState == ButtonState.PRESSED){
            robot.vSlides.moveTo(level-260);
        } else if(buttonState == ButtonState.NO){
            robot.vSlides.moveTo(level);
        }

        //Send the slides to the bottom
        if(slideGoBottom){
            slideBottom();
            goZero = false;
        }

        if(gamepad2.left_trigger > 0.9)
            slideMode = SlideMode.CONE;
         else
            slideMode = SlideMode.NORMAL;

        //Controls for slide levels
        if((gamepad1.dpad_down || gamepad2.left_bumper) && Button.BTN_BOTTOM.canPress(timestamp)){
            if(slideDown){
                robot.clawGrab();
                slideLocation = SlideLocation.BOTTOM;
                slideGoBottom = true;
            }
        } else if((gamepad2.dpad_left) && Button.BTN_L1.canPress(timestamp)){
            if(slideMode == SlideMode.NORMAL) {
                if (slideDown) {
                    robot.clawGrab();
                    robot.vSlides.moveTo1();
                    slideLocation = SlideLocation.L1;
                }
            } else {
                robot.clawOpen();
                robot.vSlides.moveTo(105);
            }
        } else if((gamepad2.dpad_down) && Button.BTN_L2.canPress(timestamp)){
            if(slideMode == SlideMode.NORMAL) {
                robot.vSlides.moveTo2();
                slideLocation = SlideLocation.L2;
            } else {
                robot.clawOpen();
                robot.vSlides.moveTo(180);
            }
        } else if((gamepad2.dpad_right) && Button.BTN_L3.canPress(timestamp)){
            if(slideMode == SlideMode.NORMAL) {
                robot.vSlides.moveTo3();
                slideLocation = SlideLocation.L3;
            } else {
                robot.clawOpen();
                robot.vSlides.moveTo(255);
            }
        } else if((gamepad2.dpad_up) && Button.BTN_L4.canPress(timestamp)){
            if(slideMode == SlideMode.NORMAL) {
                robot.vSlides.moveTo4();
                slideLocation = SlideLocation.L4;
            } else {
                robot.clawOpen();
                robot.vSlides.moveTo(330);
            }
        }

        if(gamepad1.y && Button.SET_ZERO.canPress(timestamp)){
            goZero = false;
            slideManualDown = false;
            slideGoBottom = false;
            zeroDown = false;
            robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setPower(0);
            robot.vSlides.forcestop();
        }
        //telemetry.update();
    }

    public void manualDown(){
        if(!robot.vSlides.switchSlideDown.isTouch()){
            robot.vSlides.moveToBottom();
        } else {
            robot.vSlides.forcestop();
            robot.vSlides.reset(robot.vSlides.slideLeft);
            robot.vSlides.reset(robot.vSlides.slideRight);
            telemetry.addLine("reset");
            slideManualDown = false;
        }
    }

    /**
     * Ensure that the slides safely reach the bottom with the arm in a correct position
     */
    public void slideBottom() {
        if (!robot.vSlides.switchSlideDown.isTouch() && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.vSlides.moveToBottom();
        } else {
            robot.vSlides.forcestop();
            robot.vSlides.reset(robot.vSlides.slideLeft);
            robot.vSlides.reset(robot.vSlides.slideRight);
            telemetry.addLine("reset");
            //telemetry.update();
            if(justAdjusted) {
                zeroDown = true;
                robot.turret.moveTo(turretAdjust, turretPower);
                counter++;

                if (counter > 35 || Math.abs(robot.turret.getCurrentPosition()) < Math.abs(turretAdjust)+5) {
                    robot.clawOpen();
                    clawState = ClawState.OPEN;

                    //robot.vSlides.resetSlidePosition();
                    slideGoBottom = false;
                    zeroDown = false;
                    counter = 0;
                    justAdjusted = false;
                }
            } else{
                robot.clawOpen();
                clawState = ClawState.OPEN;
                slideGoBottom = false;
            }
        }
    }

    public void brakeSlides(int level){
        if(robot.vSlides.getCurrentPosition()-level < 5){
            robot.vSlides.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.vSlides.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.vSlides.setPower(0);
        }
    }
    /*
    gamepad1:
    - joysticks = arcade drive
    - left bumper = lower slides a little
    - right bumper = claw
    - a = slow mode for drive
    - b = turn turret to right and lower
    - x = turn turret to left and lower
    - y = set powers to 0
    - up = slide all the way down (just switch)
    - right =
    - down = go to bottom
    - left =
    - left trigger = turret reset
    - right trigger =

    gamepad2:
    - left joystick = h slides
    - right joystick = turret
    - left bumper = go to bottom
    - right bumper = claw
    - a = turret go to zero and slide down
    - b = go 90 to left
    - x = go 90 to right
    - y = reset h slides
    - up = level 4
    - right = level 3
    - down = level 2
    - left = level 1
    - left trigger = cone stack mode
    - right trigger = turret and hslides slow mode
     */
}
package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;


import overcharged.components.Button;
import overcharged.components.MecanumDrive;
import overcharged.components.OcMotor;
import overcharged.components.RobotMecanum;
import overcharged.components.hSlides;
/*
    CONTROLS

    (gamepad1, main controller)
    joysticks = arcade drive
    x, b = turret 90 rotation
    y = slow mode
    right bumper = claw grab/open
    left bumper = lower slides slightly
    dpad down = level 2 slide
    dpad right = level 3 slide
    dpad up = level 4 slide
    dpad left = go to bottom
    a = toggle drive controls (only drive)

    gamepad 2 (child proof)
    joysticks = arcade drive
    y = slides up (height 1)
    a = reset/go too bottom
    b = claw toggle


 */
@Config
@TeleOp(name="outreach", group="Test")
public class outreachTele extends OpMode {
    RobotMecanum robot;
    int slideLevel = 0;
    boolean clawOpen = true;
    float turretPower = 1;
    int turretAdjust = 0;
    boolean goToBottom = false;
    double dPower = 1;
    boolean alignerOut = false;
    boolean mainController = true;
    SlideLocation slideLocation = SlideLocation.BOTTOM;
    boolean slowMode = false;
    ButtonState buttonState = ButtonState.NO2;
    TurretSide turretSide = TurretSide.NONE;
    public enum SlideLocation {
        BOTTOM,
        STACK,
        L1,
        LT,
        L2,
        L3,
        L4;
    }
    public enum ButtonState{
        PRESSED,
        PRESSING,
        NO,
        NO2;
    }
    public enum TurretSide {
        RIGHT,
        NONE,
        LEFT;

    }
    @Override
    public void init() {
        robot = new RobotMecanum(this, false, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.setBulkReadManual();
        robot.clawOpen();
        robot.vSlides.slideLeft.setPower(0);
        robot.vSlides.slideMiddle.setPower(0);
        robot.vSlides.slideRight.setPower(0);
   /*     driveLeftFront = hardwareMap.get(DcMotor.class, "driveLF");
        driveLeftBack = hardwareMap.get(DcMotor.class, "driveLB");
        driveRightFront = hardwareMap.get(DcMotor.class, "driveRF");
        driveRightBack = hardwareMap.get(DcMotor.class, "driveRB");
  */
/*
        driveLeftBack = hardwareMap.dcMotor.get("driveLB");
        driveLeftFront = hardwareMap.dcMotor.get("driveLF");
        driveRightBack = hardwareMap.dcMotor.get("driveRB");
        driveRightFront = hardwareMap.dcMotor.get("driveRF");
 */
        //robot.dr.setDirection(DcMotorSimple.Direction.REVERSE);
        //driveRightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        //robot.vSlides.moveToBottom();
        /*robot.vSlides.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.vSlides.slideMiddle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.vSlides.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.vSlides.slideLeft.setPower(-0.8f);
        robot.vSlides.slideMiddle.setPower(-0.8f);
        robot.vSlides.slideRight.setPower(-0.8f);*/
        long timestamp = System.currentTimeMillis();
        double drivePower = gamepad1.left_stick_y;;
        double turnPower= -gamepad1.right_stick_x;;
        double strafePower = -gamepad1.left_stick_x;
        double RB_Power = (-turnPower + drivePower + strafePower)*dPower;
        double RF_Power = (-turnPower + drivePower - strafePower)*dPower;
        double LF_Power = (turnPower + drivePower + strafePower)*dPower;
        double LB_Power = (turnPower + drivePower - strafePower)*dPower;

        if(mainController) {
            robot.driveLeftFront.setPower(LF_Power);
            robot.driveLeftBack.setPower(LB_Power);
            robot.driveRightFront.setPower(RF_Power);
            robot.driveRightBack.setPower(RB_Power);
        }

        double drivePower2 = gamepad2.left_stick_y;;
        double turnPower2= -gamepad2.right_stick_x;;
        double strafePower2 = -gamepad2.left_stick_x;
        double RB_Power2 = (-turnPower2 + drivePower2 + strafePower2)*dPower;
        double RF_Power2 = (-turnPower2 + drivePower2 - strafePower2)*dPower;
        double LF_Power2 = (turnPower2 + drivePower2 + strafePower2)*dPower;
        double LB_Power2 = (turnPower2 + drivePower2 - strafePower2)*dPower;

        if(!mainController) {
            robot.driveLeftFront.setPower(LF_Power2);
            robot.driveLeftBack.setPower(LB_Power2);
            robot.driveRightFront.setPower(RF_Power2);
            robot.driveRightBack.setPower(RB_Power2);
        }
        if (gamepad1.a && Button.RESET_H.canPress(timestamp)){
            if(mainController)
                mainController=false;
            else if(!mainController)
                mainController = true;
        }
        //Claw
        if((gamepad1.right_bumper||gamepad2.b) && Button.BTN_L2.canPress(timestamp)){
            if(clawOpen) {
                robot.clawGrab();
                clawOpen = false;

            }
            else if(!clawOpen){
                robot.clawOpen();
                if(turretSide == TurretSide.LEFT)
                    robot.aligner.setRight();
                else{
                    robot.aligner.setLeft();
                }
                robot.aligner.setInit();
                clawOpen = true;
            }

        }
        //SLIDES
        if((gamepad1.dpad_down || gamepad2.y) && Button.BTN_L1.canPress(timestamp)){
            robot.vSlides.moveTo2();
            slideLocation = SlideLocation.L2;
            dPower = 0.7f;
        }
        if(gamepad1.dpad_right && Button.BTN_ALIGNER.canPress(timestamp)){
            robot.vSlides.moveTo3();
            slideLocation = SlideLocation.L3;
            dPower = 0.4f;
        }
        if(gamepad1.dpad_up && Button.TURRET_RESET.canPress(timestamp)){
            robot.vSlides.moveTo4();
            slideLocation = SlideLocation.L4;
            dPower = 0.3f;
        }
        if(gamepad1.x && Button.BTN_TURRET.canPress(timestamp)) {
            robot.turret.setPower(0);
            turretAdjust = 92;
            robot.turret.moveTo(turretAdjust, turretPower);
            if(slideLocation == slideLocation.L3 || slideLocation == SlideLocation.L4){
                robot.aligner.setTeleOut();
                robot.aligner.setLeft();
                turretSide = TurretSide.LEFT;
                alignerOut = true;
            }
        }
        if(gamepad1.b && Button.TURRET_LEFT.canPress(timestamp)) {

            robot.turret.setPower(0);
            turretAdjust = -90;
            robot.turret.moveTo(turretAdjust, turretPower);
            if(slideLocation == slideLocation.L3 || slideLocation == SlideLocation.L4){
                robot.aligner.setTeleOut();
                robot.aligner.setRight();
                turretSide = TurretSide.RIGHT;
                alignerOut = true;

            }
        }

        //toggle slow mode
        if(gamepad1.y && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            if(!slowMode){
                dPower = 0.4f;
                slowMode = true;
            }
            else if(slowMode){
                dPower = 1;
                slowMode = false;
            }

        }


        if((gamepad1.dpad_left||gamepad2.a) && Button.BTN_AUTOGRAB.canPress(timestamp)){
            robot.turret.setPower(0);
            turretAdjust = 0;
            robot.turret.moveTo(turretAdjust, turretPower);
            robot.aligner.setLeft();
            robot.alignerInit();
            robot.clawGrab();
            turretSide = TurretSide.NONE;
            goToBottom = true;


        }
        if(goToBottom){
            moveBottom();


        }
        //lower slides a little
        if((gamepad1.left_bumper && slideLocation != SlideLocation.BOTTOM)){
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
        //get slide heights
        if(slideLocation != SlideLocation.BOTTOM) {
            if (slideLocation == SlideLocation.L2)
                slideLevel = robot.vSlides.level2;
            else if (slideLocation == SlideLocation.L3)
                slideLevel = robot.vSlides.level3;
            else if (slideLocation == SlideLocation.L4)
                slideLevel = robot.vSlides.level4;
        }
        if(buttonState == ButtonState.PRESSED){
            robot.vSlides.moveTo(slideLevel-195);
        }else if(buttonState == ButtonState.NO){
            robot.vSlides.moveTo(slideLevel);
        }


        telemetry.addData("limit switch ", robot.vSlides.isSlideSwitchPressed());
        telemetry.update();


    }
    //move slides to bottom
    public void moveBottom(){

        /*telemetry.addData("sped",robot.vSlides.slideLeft.getPower());
        telemetry.update();*/
        RobotLog.ii(TAG_SL, "slideL encoder sped " + robot.vSlides.slideLeft.getPower());


        if (!robot.vSlides.switchSlideDown.isTouch()){
            if(Math.abs(robot.turret.getCurrentAngle()) == 0 || !alignerOut) {

                // && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
                robot.vSlides.moveToBottom();

                RobotLog.ii(TAG_SL, "not touched " + robot.vSlides.switchSlideDown.isTouch());
            }
        }
        if (robot.vSlides.switchSlideDown.isTouch()) {
            RobotLog.ii(TAG_SL, "touched " + robot.vSlides.switchSlideDown.isTouch());
            robot.vSlides.forcestop();

            robot.vSlides.reset(robot.vSlides.slideLeft);
            robot.vSlides.reset(robot.vSlides.slideMiddle);
            robot.vSlides.reset(robot.vSlides.slideRight);
            robot.vSlides.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.vSlides.slideMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.vSlides.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(!slowMode)
                dPower = 1;
            robot.clawOpen();
            slideLocation = SlideLocation.BOTTOM;
            goToBottom = false;
            alignerOut = false;
        }


    }



}


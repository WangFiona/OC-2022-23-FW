package overcharged.opmode;

import static overcharged.config.RobotConstants.TAG_T;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.Cup;
import overcharged.components.oldRobotMecanum;

/**
 * Overcharged Team #12599
 * Robot definition for mecanum robot
 */
@Disabled
@Config
@TeleOp(name="mecanum teleop")
public class mecanumTeleop extends OpMode {

    oldRobotMecanum robot;

    // Intake States
    boolean intakeOn = false;
    boolean outTake = false;

    private final static float SLOW_POWER_MULT = 0.5f;
    boolean isSlow = false;
    private boolean retractdeposit = false;

    /*public static double p = 20;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;*/

    double level3 = 660;
    double level2 = 320;
    double SLIDE_POSITION_THRESHOLD = 5;

    //ElapsedTime time;
    long startTime;

    /**
     * Indicates the current state of the intake
     */
    IntakeStep intakeStep = IntakeStep.NONE;

    public enum IntakeStep {
        NONE,
        CUP_LOCK,
        OUTTAKE,
        DOUBLE_CHECK,
        INTAKE_OFF,
        ARM_MID,
        ARM_OUT,
        DUMP,
        RETRACT_ARM,
        RETRACT_SLIDES
    }

    public enum DuckStep {
        OFF,
        ACCELERATING,
        ON
    }
    DuckStep duckStep = DuckStep.OFF;
    long duckStartTime = 0;
    public static double duckBasePower = 0.5;
    public static double duckFinalPower = 0.69;
    public static double accelerationTime = 900;
    public static double duckElapsedTime = 0;
    public static double duckStopTime = 4000;

    boolean booleanVariableForTheIsSlideDownMethod = false;

//    LedThread ledThread;
//    //0 = running, 1 = isSlow, 2 = isBlock, 3 = isBall
//    boolean[] ledStates = new boolean[]{true, false, false, false, false};

//    boolean resetCycle = false;
//
//    RetractionStep retractionStep = RetractionStep.UP;
//    public enum RetractionStep {
//        UP,
//        ARM_MID,
//        DOWN
//    }


    @Override
    public void init() {
        try {
            RobotLog.ii(TAG_T, "Teleop init start");
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new oldRobotMecanum(this, false, false);
            RobotLog.ii(TAG_T, "Teleop init RobotMecanum initialized");
            startTime = System.currentTimeMillis();
            retractdeposit = false;
//        ledThread = new LedThread(robot.ledStrip, ledStates);
//        ledThread.run();
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
        float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = gamepad1.right_stick_y;
        float arm_y1 = -gamepad2.left_stick_y;
        float arm_y2 = -gamepad2.right_stick_y;

        //check slow mode
        if (gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = !isSlow;
//            if (isSlow) {
//            ledStates[1] = true;
//            } else  {
//            ledStates[1] = false;
//            }
        }

        if (isSlow) {
            x1 *= SLOW_POWER_MULT;
            x2 *= SLOW_POWER_MULT;
            y1 *= SLOW_POWER_MULT;
            y2 *= SLOW_POWER_MULT;
            robot.ledBlueBlink();
        } else {
            robot.ledBlueOn(false);
        }
        //robot.drive.setArcade(x1 * powerMult, y1 * powerMult, -0.67f * x2);
        if ((Math.abs(y1) < Math.abs(x1) && Math.abs(y2) < Math.abs(x2)) ||
                (Math.abs(y1) < Math.abs(x1) && x2 == 0) ||
                (Math.abs(y2) < Math.abs(x2) && x1 == 0)) {
            float avg = (x1 + x2)/2f;
            robot.drive.setStrafePower(avg, avg);
        } else {
            //straight tank
            robot.drive.setPower(y1, y2);
        }

        //show the yellow or white led everytime the block or ball is collected
        if (robot.isCollected()) {
            robot.ledGreenOn(true);
            if (robot.getFreight() == Cup.FreightType.BLOCK) {
                robot.ledYellowOn(true);
            } else if (robot.getFreight() == Cup.FreightType.BALL) {
                //robot.ledWhiteOn(true);
            } else {
                robot.ledWhiteBlink();
            }
        } else {
            robot.ledYellowOn(false);
            robot.ledWhiteOn(false);
            robot.ledGreenOn(false);
        }
        //robot.slide.setPIDFCoefficients(p,i,d,f);

        // Intake Controls
        if(gamepad1.right_trigger > 0.9 && Button.BTN_INTAKE.canPress(timestamp)){
            if(!intakeOn){
                robot.intakeOn();
                intakeOn = true;
            } else {
                robot.intakeOff();
                intakeOn = false;
            }
            outTake = false;
        }
        if(gamepad1.left_trigger > 0.9 && Button.BTN_OUTTAKE.canPress(timestamp)){
            if(!outTake){
                robot.outtake();
                outTake = true;
            } else {
                robot.intakeOff();
                outTake = false;
            }
            intakeOn = false;
        }

        if(intakeStep != IntakeStep.NONE) {
            if (intakeStep == IntakeStep.CUP_LOCK){
                startTime = System.currentTimeMillis();
                robot.cupLocked();
                intakeStep = IntakeStep.OUTTAKE;
            } else if((intakeStep == IntakeStep.OUTTAKE) && (timestamp - startTime > 150)){
                startTime = System.currentTimeMillis();
                robot.outtake();
                intakeOn = false;
                outTake = true;
                intakeStep = IntakeStep.INTAKE_OFF;
            } else if((intakeStep == IntakeStep.INTAKE_OFF) && (timestamp - startTime > 500)){
                robot.intakeOff();
                intakeOn = false;
                outTake = false;
                robot.armMid();
                intakeStep = IntakeStep.ARM_MID;
            } else if((intakeStep == IntakeStep.DUMP) && (timestamp - startTime > 300)){
                intakeStep = IntakeStep.RETRACT_ARM;
            }
            if(gamepad1.left_bumper && Button.BTN_DUMPER.canPress(timestamp)){
                startTime = System.currentTimeMillis();
                robot.cupDump();
                intakeStep = IntakeStep.DUMP;
            }
        } else if (robot.isCollected()) {
            intakeStep = IntakeStep.CUP_LOCK;
        }

        if(gamepad2.b && Button.BTN_OUT.canPress(timestamp)){
            intakeStep = IntakeStep.ARM_OUT;
            if(robot.slides.slideReachedBottom()){
                robot.armOutShared();
            } else {
                robot.armOut();
            }
        }
        if(gamepad2.x && Button.BTN_IN.canPress(timestamp)){
            intakeStep = IntakeStep.ARM_MID;
            robot.armMid();
        }

//        if(gamepad2.a && Button.BTN_AUTOMATION.canPress(timestamp)){
//            slideBottom();
//            robot.cupOpen();
//            intakeStep = IntakeStep.NONE;
//        }

        if(!robot.isArmDown()) {
            //Slide controls
            if (gamepad2.dpad_up && Button.BTN_SLIDE_UP.canPress(timestamp)) {
                //moveSlidesTo(600);
                retractdeposit = false;
                robot.slides.moveToTop();
            } else if ((gamepad2.dpad_down || gamepad2.a) && Button.BTN_SLIDE_DOWN.canPress(timestamp)) {
                retractdeposit = true;
            } else if (gamepad2.dpad_right && Button.BTN_UP.canPress(timestamp)) {
                retractdeposit = false;
                //this is done to have a clean move down
                robot.slides.moveToMid();
            } else if (arm_y1 != 0) {
                retractdeposit = false;
                robot.slides.move(arm_y1);
            } else if (!retractdeposit && robot.slides.getCurrentPosition() < 50){
                robot.slides.keep();
            }
        }
        if (retractdeposit) {
            slideBottom();
            robot.cupOpen();
            intakeStep = IntakeStep.NONE;
        }

        /*if(gamepad2.right_trigger > 0.01){
            //robot.duckOn(gamepad2.right_trigger);
            duckOperation(gamepad2.right_trigger, false);
        } else if(gamepad2.left_trigger > 0.01){
            //robot.duckOn(gamepad2.left_trigger);
            duckOperation(gamepad2.left_trigger, true);
        } else {
            robot.duckOff();
        }*/

        if(gamepad2.right_trigger > 0.9){
            if(duckStep == DuckStep.OFF){
                duckStep = DuckStep.ACCELERATING;
                duckStartTime = timestamp;
            } else {
                duckElapsedTime = timestamp-duckStartTime;
                if (duckElapsedTime > accelerationTime) duckStep = DuckStep.ON;
                if (duckElapsedTime > duckStopTime) duckStep = DuckStep.OFF;
            }
        } else {
            duckStep = DuckStep.OFF;
        }

        if(duckStep == DuckStep.ON){
            robot.duckOn(duckFinalPower);
        } else if(duckStep == DuckStep.ACCELERATING){
            double dV = duckFinalPower-duckBasePower;
            double k = duckElapsedTime/accelerationTime;
            k = k > 1 ? 1 : k;
            double pwr = duckBasePower+(dV*k);
            robot.duckOn(pwr);
        } else {
            robot.duckOff();
        }

        /// Cap Controls
        if(gamepad2.right_stick_y != 0) robot.setCapPos(gamepad2.right_stick_y); // Enabling arm-like movement on the cap arm
        if(gamepad1.y && Button.BTN_OPEN.canPress(timestamp)) {
            robot.setCapOut();
            robot.armMid();
        }
        if(gamepad1.x && Button.BTN_OPEN.canPress(timestamp)) robot.setCapIn();
        if(gamepad2.left_bumper && Button.BTN_OPEN.canPress(timestamp)) robot.setCapped();

        robot.drawLed();

//        telemetry.addData("LF, RF", robot.driveLeftFront.getCurrentPosition() + ", " + robot.driveRightFront.getCurrentPosition());
//        telemetry.addData("LB, RB", robot.driveLeftBack.getCurrentPosition() + ", " + robot.driveRightBack.getCurrentPosition());
//        telemetry.addData("DUCK", robot.duck.duck.getCurrentPosition());
//        telemetry.addData("Slide State", slideState);
//        telemetry.addData("Slide Switch", robot.isSlideSwitchPressed());
//        telemetry.addData("Slide Position", robot.getSlidePosition());
//        telemetry.addData("Intake Step", intakeStep);
//        telemetry.addData("Arm Down", robot.isArmDown());
//        telemetry.addData("Arm Out", robot.isArmOut());
//        telemetry.addData("Slow Mode", isSlow);
//        telemetry.update();

    }

    /*private void duckOperation(float x) {
        float power = 0.3F;
        if (x >= 0.7F && x < 0.9F) {
            // between 0.5 (inclusive) and 0.8 (exclusive)
            power = 0.55F;
        } else if (x >= 0.9F) {
            // between 0.8 (inclusive) and 1.0 (exclusive)
            power = 0.7F;
        }
        robot.duckOn(power);
        telemetry.addData("Duck Value", x);
        telemetry.addData("Duck Power", power);
    }*/

    private void duckOperation(float x, boolean isReverse) {
        float power = isReverse ? -0.5F: 0.5F;
        if (x > 0.9F) {
            // between 0.8 (inclusive) and 1.0 (exclusive)
            power = isReverse ? -0.8F : 0.8F;
        }
        robot.duckOn(power);
//        telemetry.addData("Duck Value", x);
//        telemetry.addData("Duck Power", power);
    }

    public void slideBottom() {
        if (!robot.slides.slideReachedBottom()){
            retractdeposit = true;
            robot.armMid();
            robot.slides.moveToBottom();
        } else {
            robot.slides.forcestop();
            //Do you want to bring the arm down when the slide reaches the bottom all the time
            robot.armDown();
            retractdeposit = false;
        }
    }
}

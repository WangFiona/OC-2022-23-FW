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
 * Teleop for Mecanum robot blue alliance
 */
@Disabled
@TeleOp(name="blue_teleop", group="Game")
public class oldteleop extends OpMode {
    ///Overcharged Robot object
    oldRobotMecanum robot;
    // Intake States
    boolean intakeOn = false;
    boolean outTake = false;
    ///Slow mode power factor
    private final static float SLOW_POWER_MULT = 0.5f;
    boolean isSlow = false;
    private boolean retractdeposit = false;

    long startTime;
    double accelPower=0.4;
    long duckSTime;
    boolean duckStop=false;
    int x=1;
    int duckCount=0;
    long time2;
    boolean duckRunning=false;
    boolean reachless = false;
    boolean atmid = false;

    /**
     * Indicates the current state of the intake
     */
    IntakeStep intakeStep = IntakeStep.NONE;

    /**
     * Intake states
     */
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

    /**
     * Duck operation states
     */
    public enum DuckStep {
        OFF,
        ACCELERATING,
        ON,
        AUTOMATE
    }
    DuckStep duckStep = DuckStep.OFF;
    double duckBasePower = 0.38;
    double duckFinalPower = 0.5;
    double accelerationTime = 1200;
    int direction = 1;
    long start;

    @Override
    public void init() {
        try {
            RobotLog.ii(TAG_T, "Teleop init start");
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            robot = new oldRobotMecanum(this, false, false);
            RobotLog.ii(TAG_T, "Teleop init RobotMecanum initialized");
            startTime = System.currentTimeMillis();
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
        float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = gamepad1.right_stick_y;
        float arm_y1 = -gamepad2.left_stick_y;
        float arm_y2 = -gamepad2.right_stick_y;

        //check slow mode
        if (gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = !isSlow;
        }

        /**
         * Turn on slow mode and blink blue indicatior for the drivers
         */
        if (isSlow) {
            x1 *= SLOW_POWER_MULT;
            x2 *= SLOW_POWER_MULT;
            y1 *= SLOW_POWER_MULT;
            y2 *= SLOW_POWER_MULT;
            robot.ledBlueBlink();
        } else {
            robot.ledBlueOn(false);
        }

        /**
         * Set Mecanum strafe or straight drive mode
         */
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

        /**
         * When the freight is collected indicate a green led and in addition
         * show the yellow or white led everytime the block or ball is collected
         */
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

        /**
         * Automate the dock operation when the y button is pressed
         */
        /*if(gamepad2.y && Button.BTN_OPEN.canPress(timestamp)) {
            start = System.currentTimeMillis();
            duckStep = DuckStep.ACCELERATING;
        }*/

        if(duckStep != DuckStep.OFF){
            ///Start the Duck operation
            duckAccelBetter();
        }
        else if(gamepad2.right_bumper){
            ///Handle situation when duck is stuck
            robot.duckOn(0.5 * direction);
        }
        else{
            robot.duckOff();
        }


        duckAccelerate(timestamp);

        /**
         * Toggle the intake Controls
         * turn intake on/off with right trigger
         * turn outtake on/off with right trigger
         */
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

        /**
         * Automate the arm and delivery once the intake is complete
         * lock the cup with the one that was collected, then outtake any remaining freights,
         * bring the arm to mid and get ready to drive
         * Then move slide up, reach arm out and deliver to the alliance hub
         */
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
            } else if((intakeStep == IntakeStep.INTAKE_OFF) && (timestamp - startTime > 300)){
                robot.intakeOff();
                intakeOn = false;
                outTake = false;
                robot.armMid();
                intakeStep = IntakeStep.ARM_MID;
            } else if((intakeStep == IntakeStep.DUMP) && (timestamp - startTime > 110)){
                robot.cupDump();
                intakeStep = IntakeStep.RETRACT_ARM;
            }
            if(gamepad1.left_bumper && Button.BTN_DUMPER.canPress(timestamp)){
                startTime = System.currentTimeMillis();
                if(robot.arm.armOut) robot.cupDump();
                else robot.armOut();
                intakeStep = IntakeStep.DUMP;
            }
        } else if (robot.isCollected()) {
            intakeStep = IntakeStep.CUP_LOCK;
        }

        /**
         * Automate the shared hub delivery, move slide up, arm out and delivery
         */
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

        /**
         * Perform the slide operations only when the arm is not down
         */
        if(!robot.isArmDown()) {
            //Slide controls
            if(gamepad2.right_trigger != 0){
                robot.slides.move(gamepad2.right_trigger);
            }
            else if(gamepad2.left_trigger != 0){
                robot.slides.move(-gamepad2.left_trigger);
            }
            else if (gamepad2.dpad_up && Button.BTN_SLIDE_UP.canPress(timestamp)) {
                //moveSlidesTo(600);
                retractdeposit = false;
                robot.slides.moveToTop();
                atmid = false;
                reachless = false;
            } else if ((gamepad2.dpad_down || gamepad2.a || gamepad1.dpad_up || gamepad1.y) && Button.BTN_SLIDE_DOWN.canPress(timestamp)) {
                retractdeposit = true;
                atmid = false;
                reachless = false;
            } else if (gamepad2.dpad_right && Button.BTN_UP.canPress(timestamp)) {
                retractdeposit = false;
                //this is done to have a clean move down
                robot.slides.moveToMid();
                atmid = true;
                reachless = false;
            } else if(gamepad2.dpad_left && Button.BTN_UP.canPress(timestamp)){
                retractdeposit = false;
                if(!atmid){
                    robot.slides.moveToMid();
                    atmid = true;
                }
                if(!reachless){
                    robot.armOutSharedReach();
                    reachless = true;
                } else {
                    robot.armOutSharedReachLess();
                    reachless = false;
                }

            } else if (!retractdeposit && robot.slides.getCurrentPosition() < 50){
                robot.slides.keep();
            }
        }

        /**
         * Also have a manual backup in case automation fails for arm control
         */
        if (arm_y1 != 0) {
            //retractdeposit = false;
            //robot.slides.move(arm_y1);
            robot.arm.armDown = false;
            robot.arm.move(arm_y1);
            telemetry.addData("Arm left number: ", robot.arm.left.getPosition());
            telemetry.addData("Arm right number: ", robot.arm.right.getPosition());
            telemetry.update();
        }

        if (retractdeposit) {
            slideBottom();
            robot.cupOpen();
            intakeStep = IntakeStep.NONE;
        }

        /// Cap Controls
        if(gamepad2.right_stick_y != 0) robot.setCapPos(gamepad2.right_stick_y); // Enabling arm-like movement on the cap arm
        if(gamepad1.y && Button.BTN_OPEN.canPress(timestamp)) {
            //robot.setCapOut();
            robot.armMid();
        }
        if(gamepad1.b && Button.BTN_MIN.canPress(timestamp)) {
            robot.setCapOut();
            //robot.armMid();
        }
        if(gamepad1.a && Button.BTN_MINUS.canPress(timestamp)) {
            robot.armDown();
            //robot.armMid();
        }
        if(gamepad1.x && Button.BTN_OPEN.canPress(timestamp)) robot.setCapIn();
        if(gamepad2.left_bumper && Button.BTN_OPEN.canPress(timestamp)) robot.setCapped();

        robot.drawLed();

//        telemetry.addData("servo pos", robot.cap.cap.getPosition());
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

    /**
     * Ensure that the slides safely reach the bottom with the arm in a correct position
     */
    public void slideBottom() {
        if (!robot.slides.slideReachedBottom()){
            retractdeposit = true;
            robot.armMid();
            robot.slides.moveToBottom();
        } else {
            robot.slides.forcestop();
            //Do you want to bring the arm down when the slide reaches the bottom all the time
            if(!robot.isCollected()){
                robot.armDown();
            }
            retractdeposit = false;

        }
    }

    /**
     * Automate the duck operation for better performance
     * Accelerate for the 2/3 of the operation time and the speed up the power
     */
    public void duckAccelBetter(){
        long elapsed = System.currentTimeMillis() - start;

        if(duckStep == DuckStep.ACCELERATING){
            double dV = duckFinalPower-duckBasePower;
            double k = elapsed/accelerationTime;
            k = k > 1 ? 1 : k;
            double pwr = duckBasePower+(dV*k);
            robot.duckOn(pwr * direction);
            if(elapsed > 1000) duckStep = DuckStep.ON;
        } else if(duckStep == DuckStep.ON){
            robot.duckOn(duckFinalPower * direction);
            if (elapsed > 1500) {
                robot.duckOff();
                duckStep = DuckStep.OFF;
            }
        } else {
            robot.duckOff();
        }
    }

    public void duckAccelerate(long timestamp){
        //Duck acceleration
        if(gamepad2.y && Button.BTN_MIN.canPress(timestamp)){
            if(duckRunning==false){ //duckStep == DuckStep.AUTOMATE
                duckStep = DuckStep.OFF;
                duckRunning=true;
                accelPower=0.4;
                duckSTime=System.currentTimeMillis();
                time=System.currentTimeMillis();
                time2=System.currentTimeMillis();
                duckCount=0;
                x=1;
            } else{
                robot.duckOff();
                duckStep = DuckStep.AUTOMATE;
                duckRunning=false;
            }
        }

        int runTime=1600;
        int waitTime=500;
        if(duckRunning==true){
            if((System.currentTimeMillis()-duckSTime)>=(runTime)*x+waitTime*(x-1) && (System.currentTimeMillis()-duckSTime)<((runTime)+waitTime)*x){
                duckStop=true;
                accelPower=0;
                telemetry.addLine("Break");
            }

            if (duckStop==true && (System.currentTimeMillis()-time2) >= (waitTime+runTime)) {
                duckStop=false;
                accelPower=0.35;
                time2=System.currentTimeMillis();
                x++;
                duckCount++;
                telemetry.addLine("Start again");
            }

            if(duckStop==false && (System.currentTimeMillis()-duckSTime)>= (long) (runTime - 300) *x+ (long) waitTime *(x-1)+300*(x-1)){
                accelPower=0.55;
                telemetry.addLine("max power");

            }
            else if(duckStop==false && (System.currentTimeMillis()-time) >= 600) {
                accelPower=accelPower+0.003;
                time = System.currentTimeMillis();
                telemetry.addLine("Add acceleration");
            }

            telemetry.addData("speed: ", accelPower);
            robot.duckOn(accelPower*direction);
        }
    }
}
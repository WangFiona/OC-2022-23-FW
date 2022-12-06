package overcharged.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import overcharged.components.Button;
import overcharged.components.OcServo;

import static overcharged.components.Button.BTN_AUTO;
import static overcharged.components.Button.BTN_COLLECT;
import static overcharged.components.Button.BTN_FLYWHEEL;
import static overcharged.components.Button.BTN_REJECT;
import static overcharged.components.Button.BTN_SLIDE;
import static overcharged.components.Button.BTN_SLIDE_DOWN;
import static overcharged.components.Button.BTN_SLIDE_UP;

@Disabled
@Config
@TeleOp(name="FlywheelTester", group = "Game")
public class FlywheelTester extends OpMode {
    DcMotorEx flywheel_motor1;
    DcMotorEx flywheel_motor2;
    PIDFCoefficients flywheel_pid;
    PIDFCoefficients intake_pid;

    DcMotorEx intakeL;
    DcMotorEx intakeR;
    OcServo stopper;
    float intake_power = 1f;

    //(1000,1,100,5000)
    public static int p = 1;
    public static int i = 0;
    public static int d = 0;
    public static int f = 0;

    public static double flywheel_power = 0.68;

    boolean boolSlideUp = false;

    float stopperOpen = 150f;
    float stopperClosed = 27f;
    long outtakeAutomationTime = 0;
    boolean outtakeAutomationStart = false;

    boolean stopperDone = false;

    OcServo slide;
    float slideUp = 97f;
    float slideDown = 173f;
    float slidePos1 = 142f;
    float slidePos2 = 120f;

    public enum ValueAdjuster {
        P,
        I,
        D,
        F
    }
    ValueAdjuster coef = ValueAdjuster.P;

    public enum FlywheelState {
        ON,
        OFF
    }
    private FlywheelState flywheelState = FlywheelState.OFF;

    public enum IntakeState {
        ///Collected
        IN,
        ///Reject/outtake
        OUT,
        ///Stop intake
        STOP
    }
    private IntakeState collectionState = IntakeState.STOP;
    private IntakeState prevCollectionState = collectionState;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slide = new OcServo(hardwareMap, "slideServo", slideDown);

        flywheel_pid = new PIDFCoefficients(p, i, d, f);

        flywheel_motor1 = hardwareMap.get(DcMotorEx.class, "fm1");
        flywheel_motor2 = hardwareMap.get(DcMotorEx.class, "fm2");

        flywheel_motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel_motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel_motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel_motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pid);
        flywheel_motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pid);

        flywheel_motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel_motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        stopper = new OcServo(hardwareMap, "stopperServo", stopperClosed);
        intakeL = hardwareMap.get(DcMotorEx.class, "intakeL");
        intakeL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(DcMotorEx.Direction.FORWARD);

        intakeR = hardwareMap.get(DcMotorEx.class, "intakeR");
        intakeR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeR.setDirection(DcMotorEx.Direction.REVERSE);

        intake_pid = new PIDFCoefficients(800,20,20,5);
        intakeL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intake_pid);
        intakeR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, intake_pid);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        if(gamepad1.a && BTN_FLYWHEEL.canPress(timestamp)){
            if(flywheelState == FlywheelState.OFF){
                setFlywheelPower(flywheel_power);
                flywheelState = FlywheelState.ON;
            } else if (flywheelState == FlywheelState.ON){
                setFlywheelPower(0);
                flywheelState = FlywheelState.OFF;
            }
        }

        if(gamepad1.dpad_down && !outtakeAutomationStart && BTN_AUTO.canPress(timestamp)){
            outtakeAutomationTime = System.currentTimeMillis();
            outtakeAutomationStart = true;
            stopperDone = true;
            stopper.setPosition(stopperOpen);
        }
        if(outtakeAutomationStart && stopperDone && System.currentTimeMillis()-outtakeAutomationTime > 600){
            outtakeAutomationTime = System.currentTimeMillis();
            boolSlideUp = true;
            stopperDone = false;
            slide.setPosition(slideUp);
        }
        if(outtakeAutomationStart && boolSlideUp && System.currentTimeMillis()-outtakeAutomationTime > 650){
            slide.setPosition(slideDown);
            stopper.setPosition(stopperClosed);
            outtakeAutomationStart = false;
            boolSlideUp = false;
            if(flywheelState==FlywheelState.ON){
                setFlywheelPower(0);
                flywheelState=FlywheelState.OFF;
            }
        }
        if (gamepad1.right_trigger > 0.9 && BTN_COLLECT.canPress(timestamp)) {
            if (collectionState == IntakeState.IN) {
                collectionState = IntakeState.STOP;
            } else {
                collectionState = IntakeState.IN;
            }
        } else if (gamepad1.left_trigger > 0.9 && BTN_REJECT.canPress(timestamp)) {
            if (collectionState == IntakeState.OUT) {
                collectionState = IntakeState.STOP;
            } else {
                collectionState = IntakeState.OUT;
            }
        }

        setIntake(collectionState);
        if(gamepad1.b && Button.BTN_DUMPER.canPress(timestamp)){
            if(coef == ValueAdjuster.P) coef = ValueAdjuster.I;
            else if(coef == ValueAdjuster.I) coef = ValueAdjuster.D;
            else if(coef == ValueAdjuster.D) coef = ValueAdjuster.F;
            else if(coef == ValueAdjuster.F) coef = ValueAdjuster.P;
        }
        if(gamepad1.x && Button.BTN_DUMPER.canPress(timestamp)){
            if(coef == ValueAdjuster.P) coef = ValueAdjuster.F;
            else if(coef == ValueAdjuster.F) coef = ValueAdjuster.D;
            else if(coef == ValueAdjuster.D) coef = ValueAdjuster.I;
            else if(coef == ValueAdjuster.I) coef = ValueAdjuster.P;
        }
        telemetry.addData("P", p);
        telemetry.addData("I", i);
        telemetry.addData("D", d);
        telemetry.addData("F", f);
        telemetry.addData("Current Power", flywheel_power);
        telemetry.addData("Current Velocity", flywheel_motor1.getVelocity());
        adjustValue(timestamp);
    }

    public void adjustValue(long timestamp){
        if(gamepad1.right_bumper && Button.BTN_DUMPER.canPress(timestamp)) {
            if(coef == ValueAdjuster.P) p += 50;
            else if(coef == ValueAdjuster.I) i += 5;
            else if(coef == ValueAdjuster.D) d += 1;
            else if(coef == ValueAdjuster.F) f += 1;
            flywheel_pid = new PIDFCoefficients(p,i,d,f);
        } else if(gamepad1.left_bumper && Button.BTN_DUMPER.canPress(timestamp)) {
            if(coef == ValueAdjuster.P) p -= 50;
            else if(coef == ValueAdjuster.I) i -= 5;
            else if(coef == ValueAdjuster.D) d -= 1;
            else if(coef == ValueAdjuster.F) f -= 1;
            flywheel_pid = new PIDFCoefficients(p,i,d,f);
        }
    }

    private void setIntake(IntakeState intakeState) {
        //motor power
        float pwr = intake_power;

        switch (intakeState) {
            case IN:
                intakeL.setPower(pwr);
                intakeR.setPower(pwr);
                break;
            case OUT:
                intakeL.setPower(-0.6*pwr);
                intakeR.setPower(-0.6*pwr);
                break;
            case STOP:
                intakeL.setPower(0);
                intakeR.setPower(0);
            default:
                intakeState = IntakeState.STOP;
                break;
        }
        prevCollectionState = intakeState;
    }

    private void setFlywheelPower(double pwr){
        flywheel_motor1.setPower(pwr);
        flywheel_motor2.setPower(pwr);
    }

    /*private void setFlywheelPower(float pwr){
        flywheel_motor1.setVelocity(2150*pwr);
        flywheel_motor2.setVelocity(2150*pwr);
    }*/
}

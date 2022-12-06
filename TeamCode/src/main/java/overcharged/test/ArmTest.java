package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.OcServo;

/**
 * Created by Parthiv Nair on 10/13/2018.
 * You can set the initial position using this program. This is useful especially when replacing servos
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ArmTest", group = "Test")
public class ArmTest extends OpMode {
    private Servo left;
    private Servo right;

    private ElapsedTime runtime = new ElapsedTime();
    private final static int MIN_SERVO_TICK = 1;

    @Override
    public void init() {
        hardwareMap.logDevices();
    }

    double outSharedAutoR = 0.643;
    double outSharedAutoL = 0.427;
    double autoOutforR = 0.621;
    double autoOutforL = 0.449;
    double downR = 0.265;
    double downL = 0.805;

    /*int position = 127;
    int max = 255;*/
    boolean initialized = false;
    boolean servoInitialized = false;

    private void beforeInitialization(long timeStamp)
    {

    }

    private void initializeServo()
    {
        servoInitialized = true;
        RobotLog.e("Initializing servo");
        //Initialize Servo
        try {
            right = hardwareMap.servo.get("armR");
            left = hardwareMap.servo.get("armL");
            //telemetry.addData("Status", "Servo initialized at " + position);
        } catch (Exception e) {
            telemetry.addData("Status", "missing: servo at port 0 " + e.getMessage());
        }
        telemetry.update();
    }

    private void afterInitialization(long timeStamp)
    {
        if (!servoInitialized) {
            initializeServo();
        }
        //telemetry.addData("Position", Integer.toString(position));

        ///Change servo position for calibration
        if(gamepad1.y && Button.BTN_BACK.canPress4Short(timeStamp)){
            left.setPosition(outSharedAutoL);
            right.setPosition(outSharedAutoR);
        } else if(gamepad1.a && Button.BTN_PLUS.canPress4Short(timeStamp)){
            left.setPosition(autoOutforL);
            right.setPosition(autoOutforR);
        } else if(gamepad1.b && Button.BTN_MID.canPress(timeStamp)) {
            left.setPosition(outSharedAutoL);
            right.setPosition(outSharedAutoR);
        }

        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Select", "x:-, b:+, y:max(255), a:min(0), RightStick:mid(128)");
        long timeStamp = System.currentTimeMillis();
        afterInitialization(timeStamp);

        /*if (initialized) {
            afterInitialization(timeStamp);
        } else {
            beforeInitialization(timeStamp);
        }*/
        telemetry.update();
    }
}
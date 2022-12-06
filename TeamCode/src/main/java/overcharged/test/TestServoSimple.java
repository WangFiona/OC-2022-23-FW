package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.OcServo;

/**
 * Created by Parthiv Nair on 10/13/2018.
 * You can set the initial position using this program. This is useful especially when replacing servos
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestServoSimple", group = "Tester")
public class TestServoSimple extends OpMode {
    private OcServo servo;
    private ElapsedTime runtime = new ElapsedTime();
    private final static int MIN_SERVO_TICK = 1;

    @Override
    public void init() {
        hardwareMap.logDevices();
        RobotLog.e("Initializing servo");
        //Initialize Servo
        try {
            servo = new OcServo(hardwareMap, "servo", position);
            telemetry.addData("Status", "Servo initialized at " + position);
        } catch (Exception e) {
            telemetry.addData("Status", "missing: servo at port 0 " + e.getMessage());
        }
        telemetry.update();
    }

    float position = 127f;

    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Select", "x:-, b:+, y:max(255), a:min(0), RightStick:mid(128)");
        long timeStamp = System.currentTimeMillis();
        telemetry.addData("Position", Float.toString(position));

        ///Change servo position for calibration
        if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
            position -= MIN_SERVO_TICK;
        } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
            position += MIN_SERVO_TICK;
        } else if (gamepad1.y && Button.BTN_MAX.canPress(timeStamp)) {
            position = 255;
        } else if (gamepad1.a && Button.BTN_MIN.canPress(timeStamp)) {
            position = 0;
        } else if (gamepad1.right_stick_button && Button.BTN_MID.canPress(timeStamp)) {
            position = 128;
        }
        position = Range.clip(position, 0, 255);
        servo.setPosition(position);

        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }

        telemetry.update();
    }
}
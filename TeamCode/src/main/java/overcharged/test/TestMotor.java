package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.OcMotor;

/**
 * Created by Parthiv Nair on 1/30/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestMotor", group = "Tester")
public class TestMotor extends OpMode {
    public OcMotor motor;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();
    private MotorState motorState = MotorState.OFF;

    public enum MotorState{
        ON,
        OFF,
        GOZERO,
    }

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Motors
        try {
            motor = new OcMotor(hardwareMap,
                    "turret",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.resetPosition();
            motor.setPower(0f);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    float power = 0f;
    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Test", "Left Stick, x=0.3, y=0.5, b=0.7, a=0.9");
        telemetry.addData("Stop", "Back");
        if (gamepad1.x && Button.BTN_UP.canPress(timestamp)) {
            power = 0.3f;
        } else if (gamepad1.y && Button.BTN_UP.canPress(timestamp)) {
            power = 0;
        } else if (gamepad1.b && Button.BTN_UP.canPress(timestamp)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setTargetPosition(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.7);
            motorState = MotorState.GOZERO;
        } else if (gamepad1.a && Button.BTN_UP.canPress(timestamp)) {
            power = -0.3f;
        } else {
            //power = gamepad1.left_stick_y;
        }
        //power = Range.clip(power, -1f, 1f);

        if(motorState != MotorState.GOZERO)
            motor.setPower(power);
        telemetry.addData("Encoder", numberFormatter.format(motor.getCurrentPosition()));
        telemetry.addData("Power", power);
        telemetry.addData("Stop", "Back");
        long timeStamp = System.currentTimeMillis();
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
    }
}

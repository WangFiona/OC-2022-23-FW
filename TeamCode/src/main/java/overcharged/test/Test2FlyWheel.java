package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.OcMotorEx;

/**
 * Created by Parthiv Nair on 9/28/2020.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test2FlyWheel", group = "Tester")
public class Test2FlyWheel extends OpMode {
    public OcMotorEx motorL;
    public OcMotorEx motorR;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Motors
        try {
            motorL = InitializeMotor("motorL", DcMotorEx.Direction.REVERSE);
            motorR = InitializeMotor("motorR", DcMotorEx.Direction.REVERSE);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    private OcMotorEx InitializeMotor(String id, DcMotorEx.Direction direction) {
        OcMotorEx motor = new OcMotorEx(hardwareMap,
                id,
                direction, DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.resetPosition();
        motor.setPower(0f);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }

    float power = 0f;
    boolean vel = false;
    double motorVelocity = 0;
    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        if (gamepad1.left_bumper && Button.BTN_UP.canPress(timestamp)) {
            vel = !vel;
            power = 0f;
            motorL.setPower(power);
            motorR.setPower(power);
            motorVelocity = 0;
        }
        if (gamepad1.right_bumper) {
            motorVelocity++;
        }
        if (gamepad1.x && Button.BTN_UP.canPress(timestamp)) {
            power = 0.4f;
            // This will turn the motor at 960 ticks per second
            motorVelocity = 960;
        } else if (gamepad1.y && Button.BTN_UP.canPress(timestamp)) {
            power = 0.6f;
            // This will turn the motor at 1440 ticks per second
            motorVelocity = 1440;
        } else if (gamepad1.b && Button.BTN_UP.canPress(timestamp)) {
            power = 0.8f;
            // This will turn the motor at 1920 ticks per second
            motorVelocity = 1920;
        } else if (gamepad1.a && Button.BTN_UP.canPress(timestamp)) {
            power = 1f;
            // This will turn the motor at 200 ticks per second
            //2460 or 2480 is the max for Rev ultra HD HEx motor
            motorVelocity = 2400;
        } else {
            //power = gamepad1.left_stick_y;
        }
        //power = Range.clip(power, -1f, 1f);
        if (vel) {
            telemetry.addData("Status", "Velocity: " + runtime.toString());
            motorL.setVelocity(motorVelocity);
            motorR.setVelocity(motorVelocity);
        } else {
            telemetry.addData("Status", "Power: " + runtime.toString());
            motorL.setPower(power);
            motorR.setPower(power);
        }
        telemetry.addData("Velocity", "Left=" + numberFormatter.format(motorL.getVelocity()) + " Right=" + numberFormatter.format(motorR.getVelocity()));
        telemetry.addData("Encoder", "Left=" + numberFormatter.format(motorL.getCurrentPosition()) + " Right=" + numberFormatter.format(motorR.getCurrentPosition()));
        telemetry.addData("Power", "Left=" + numberFormatter.format(motorL.getPower()) + " Right=" + numberFormatter.format(motorR.getPower()));
        telemetry.addData("Test", "LeftBumper(Velocity/Power), RB(Velocity++) x=0.4, y=0.6, b=0.8, a=1");
        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timestamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }

//        // Loop while the motor is moving to the target
//        while(motor.isBusy()) {
//            // Let the drive team see that we're waiting on the motor
//            telemetry.addData("Status", "Waiting for the motor to reach its target");
//            telemetry.update();
//        }
//        // The motor has reached its target position, and the program will continue

        telemetry.update();
    }
}

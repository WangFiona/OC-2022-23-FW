package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.OcMotorEx;

/**
 * Created by Parthiv Nair on 10/2/2020.
 */

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test1FlyWheelPower", group = "Tester")
public class Test1FlyWheelPower extends OpMode {
    public OcMotorEx motorL;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######.##");
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Motors
        try {
            motorL = InitializeMotor("motorL", DcMotorEx.Direction.REVERSE);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    private OcMotorEx InitializeMotor(String id, DcMotorEx.Direction direction) {
        OcMotorEx motor = new OcMotorEx(hardwareMap,
                id,
                direction, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.resetPosition();
        motor.setPower(0f);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    float power = 0f;
    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        if (gamepad1.left_bumper && Button.BTN_UP.canPress(timestamp)) {
            power = power - 0.01f;
        }
        if (gamepad1.right_bumper  && Button.BTN_UP.canPress(timestamp)) {
            power = power + 0.01f;
        }
        if (gamepad1.x && Button.BTN_UP.canPress(timestamp)) {
            power = 0.4f;
        } else if (gamepad1.y && Button.BTN_UP.canPress(timestamp)) {
            power = 0.6f;
        } else if (gamepad1.b && Button.BTN_UP.canPress(timestamp)) {
            power = 0.8f;
        } else if (gamepad1.a && Button.BTN_UP.canPress(timestamp)) {
            power = 1f;
        }
        power = Range.clip(power, -1f, 1f);
        motorL.setPower(power);
        telemetry.addData("Test", "LB(Power--), RB(Power++) x=0.4, y=0.6, b=0.8, a=1");
        telemetry.addData("Power (" + power + ")", motorL.getPower());
        telemetry.addData("Velocity", numberFormatter.format(motorL.getVelocity()));
        telemetry.addData("Encoder", motorL.getCurrentPosition());
        telemetry.update();
    }
}

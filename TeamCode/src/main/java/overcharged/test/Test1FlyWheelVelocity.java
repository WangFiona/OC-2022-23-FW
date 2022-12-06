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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test1FlyWheelVelocity", group = "Tester")
public class Test1FlyWheelVelocity extends OpMode {
    public OcMotorEx motorL;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
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
                direction, DcMotorEx.RunMode.RUN_USING_ENCODER);
//        //Do this only for HD Hex motor
//        //DONOT do this for Core Hex motor
//        motor.setVelocityPIDFCoefficients(1.17, 0.117, 0, 11.7);
        motor.resetPosition();
        motor.setPower(0f);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }

    double motorVelocity = 0;
    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        if (gamepad1.left_bumper && Button.BTN_UP.canPress(timestamp)) {
            motorVelocity = motorVelocity + 10;
        }
        if (gamepad1.right_bumper  && Button.BTN_UP.canPress(timestamp)) {
            motorVelocity = motorVelocity - 10;
        }
       // motorVelocity = Range.clip(motorVelocity, 0, 2400);
        if (gamepad1.x && Button.BTN_UP.canPress(timestamp)) {
            // This will turn the motor at 960 ticks per second RADIANS -12.34/-12.56 DEGREES -694/-707
            motorVelocity = -700; //DEGREES
        } else if (gamepad1.y && Button.BTN_UP.canPress(timestamp)) {
            // This will turn the motor at 1440 ticks per second RADIANS -18.400/-18.625 DEGREES -1041/-1054
            motorVelocity = -1050; //DEGREES
        } else if (gamepad1.b && Button.BTN_UP.canPress(timestamp)) {
            // This will turn the motor at 1920 ticks per second RADIANS -24.68/-24.90 DEGREES -1414/-1427
            motorVelocity = -1420; //DEGREES
        } else if (gamepad1.a && Button.BTN_UP.canPress(timestamp)) {
            // This will turn the motor at 200 ticks per second
            //2460 or 2480 is the max for Rev ultra HD HEx motor RADIANS -30.29/-30.59 DEGREES -1722/-1735
            motorVelocity = -1730; //DEGREES
        }
        motorL.setVelocity(motorVelocity);
        telemetry.addData("Test", "LB(Velocity--), RB(Velocity++) x=960, y=1440, b=1920, a=2400");
        telemetry.addData("Power", motorL.getPower() + " Encoder=" + motorL.getCurrentPosition());
        telemetry.addData("Velocity (" + motorVelocity + ") RADIANS=", motorL.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Velocity DEGREES=", motorL.getVelocity(AngleUnit.DEGREES));
        telemetry.update();
    }
}

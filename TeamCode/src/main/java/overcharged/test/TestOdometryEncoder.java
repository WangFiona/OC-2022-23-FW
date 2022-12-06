package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import overcharged.components.Button;
import overcharged.linear.components.OcEncoder;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestOdometryEncoder", group = "Test")
public class TestOdometryEncoder extends OpMode {

    public OcEncoder encoder;

    @Override
    public void init() {
        encoder = new OcEncoder(hardwareMap,"motorL", DcMotorEx.Direction.FORWARD);
        encoder.reset();
        telemetry.addData("Status: ", "Initialized");
    }

    @Override
    public void loop() {
        long timeStamp = System.currentTimeMillis();
        telemetry.addData("reset", "a");
        if (gamepad1.a && Button.BTN_DOWN.canPress(timeStamp)) {
            encoder.reset();
        }
        telemetry.addData("Inches", encoder.getDistance());
        telemetry.addData("Encoder", encoder.getCurrentPosition());
        telemetry.update();
    }
}
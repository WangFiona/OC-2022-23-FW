package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.OcMotor;

/**
 * Created by Parthiv Nair on 10/31/2020.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestOneMotorPerSide", group = "Tester")
public class TestOneMotorPerSide extends OpMode {
    public OcMotor motor;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Motors
        try {
            motor = InitMotor("driveRF");
            this.gamepad1.reset();
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    private OcMotor InitMotor(String name) {
        OcMotor motor = new OcMotor(hardwareMap,
                name,
                DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.resetPosition();
        motor.setPower(0f);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return motor;
    }

    float power = 0f;
    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        // is reversed by default
        float y1 = -gamepad1.left_stick_y;
        // is reversed by default
        float y2 = -gamepad1.right_stick_y;
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);
        power = Range.clip((y1+y2)/2, -1f, 1f);;
        motor.setPower(power);
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Encoder", motor.getId()+ " Motor=" + motor.getCurrentPosition() );
        telemetry.addData("Power", power);
        telemetry.update();
    }
}

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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestBothMotorsPerSide", group = "Tester")
public class TestBothMotorsPerSide extends OpMode {
    public OcMotor motor1;
    public OcMotor motor2;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Motors
        try {
            motor1 = InitMotor("driveLF", DcMotor.Direction.FORWARD);
            motor2 = InitMotor("driveLB", DcMotor.Direction.REVERSE);
            this.gamepad1.reset();
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    private OcMotor InitMotor(String name, DcMotor.Direction direction) {
        OcMotor motor = new OcMotor(hardwareMap,
                name,
                direction, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        motor1.setPower(power);
        motor2.setPower(power);
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Encoder", motor1.getId()+ " Motor1=" + motor1.getCurrentPosition() + motor2.getId()+ " Motor2=" + motor1.getCurrentPosition());
        telemetry.addData("Power", power);
        telemetry.update();
    }
}

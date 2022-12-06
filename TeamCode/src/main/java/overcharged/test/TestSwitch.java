package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import overcharged.components.Button;
import overcharged.components.OcSwitch;

/**
 * Created by Parthiv Nair on 12/24/2018.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestSwitch", group = "Tester")
public class TestSwitch extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private OcSwitch ocSwitch;

    @Override
    public void init() {
        hardwareMap.logDevices();
        //Initialize Switch
        try {

            ocSwitch = new OcSwitch(hardwareMap,
                    "limitswitch",
                    true);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        long timeStamp = System.currentTimeMillis();

        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Test", "Touch the switch");
        telemetry.addData("Stop", "Back");

        boolean isTouch = ocSwitch.isTouch();

        telemetry.addData("isSlideIn", isTouch);
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
            telemetry.update();
        }
        telemetry.update();
    }
}

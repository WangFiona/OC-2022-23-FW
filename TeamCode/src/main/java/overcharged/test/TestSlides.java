package overcharged.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import overcharged.components.Button;
import overcharged.components.oldSlides;
import overcharged.util.PIDCalculator;

/**
 * Created by Parthiv Nair on 12/10/2019.
 */

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestSlides", group = "Test")
public class TestSlides extends OpMode {
    ///Declare the slides component
    private oldSlides oldSlides;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private ElapsedTime runtime = new ElapsedTime();
    ///Initialize the PIDCalculator
    private PIDCalculator pidController = new PIDCalculator(0.25, 0, 0);
    private boolean moveslidetobottom = false;
    @Override
    public void init() {
        hardwareMap.logDevices();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //Initialize Motors
        try {
            oldSlides = new oldSlides(null, hardwareMap, null);
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "Error: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Test", "Use gamepad2 dpad U/R/D and left joystick");
        telemetry.addData("Stop", "Back");
        float arm_y1 = -gamepad2.left_stick_y;
        //arm_y1 = Button.scaleInput(arm_y1);

        //Slide controls
        if (gamepad2.dpad_up && Button.BTN_SLIDE_UP.canPress(timestamp)) {
            //moveSlidesTo(600);
            moveslidetobottom = false;
            oldSlides.moveToTop();
        } else if (gamepad2.dpad_down && Button.BTN_SLIDE_DOWN.canPress(timestamp)) {
            moveslidetobottom = true;
//            slides.moveToBottom();
        } else if (gamepad2.dpad_right && Button.BTN_UP.canPress(timestamp)) {
            moveslidetobottom = false;
            oldSlides.moveToMid();
        } else if (arm_y1 != 0) {
            moveslidetobottom = false;
            oldSlides.move(arm_y1);
        } else if (!moveslidetobottom){
            oldSlides.keep();
        }
        if (moveslidetobottom) {
            oldSlides.moveToBottom();
        }
        telemetry.addData("Encoder Left", numberFormatter.format(oldSlides.slideLeft.getCurrentPosition()));
        telemetry.addData("Encoder Right", numberFormatter.format(oldSlides.slideRight.getCurrentPosition()));
        telemetry.addData("Current Left", numberFormatter.format(oldSlides.currentPositionL));
        telemetry.addData("Current Right", numberFormatter.format(oldSlides.currentPositionR));
        telemetry.addData("Run Mode", oldSlides.slideLeft.getMode());
        telemetry.addData("Velocity Left", numberFormatter.format(oldSlides.slideLeft.getVelocity()));
        telemetry.addData("Velocity Right", numberFormatter.format(oldSlides.slideRight.getVelocity()));
        telemetry.addData("Stop", "Back");
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timestamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
    }
}

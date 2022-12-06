package overcharged.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import overcharged.components.Button;
import overcharged.components.RobotMecanum;
import overcharged.components.oldRobotMecanum;

@Config
@TeleOp(name="switchTester", group="Test")
public class switchTester extends LinearOpMode {
    private RobotMecanum robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMecanum(this, true, false);
        waitForStart();
        while(opModeIsActive()){
            switchTest();
        }

        idle();
    }

    private void switchTest () {
        robot.vSlides.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.vSlides.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Switches");
            telemetry.addData(robot.vSlides.switchSlideDown.toString(), Boolean.toString(robot.vSlides.switchSlideDown.isTouch()));
            /*for (OcSwitch s : robot.switchs) {
                telemetry.addData(s.toString(), Boolean.toString(s.isTouch()));
            }*/

            telemetry.addData("Back", "LeftStick");

            telemetry.update();
            idle();
        }
        }
}

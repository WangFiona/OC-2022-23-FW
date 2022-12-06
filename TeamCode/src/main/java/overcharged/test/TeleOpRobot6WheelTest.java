package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.Button;
import overcharged.components.Robot6WheelTest;
import overcharged.components.TankDrive;

import static overcharged.config.RobotConstants.TAG_T;

/**
 * Overcharged Team #12599 TeleOp
 * This program is used for running the TeleOp.
 *
 * Automate the stone grabbing using the bottom color sensor.
 * When the stone is collected, stop the intake and grab the stone automatically and get reday for delivery
 *
 * When the slides move up the four bar automatically comes out for delivery
 * When the four bar is out the slide down power is reduced to decrease the chance of breaking the tower.
 * Use PID on the slide to keep it up at a stable position up and down.
 *
 * Driver can choose between slow, superslow and normal speed driving mode Overcharged tank drive program has the ability to navigate with snake mode.
 *
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TestTeleOp", group="Game")
public class TeleOpRobot6WheelTest extends OpMode {

    ///Overcharged 6 wheel robot
    private Robot6WheelTest robot;
    ///Overcharged Tank Drive class
    private TankDrive drive;

    private boolean isSlow = false;
    ///Initialize the Super Slow mode to false
    private boolean isSuperSlow = false;

    private static float NORMAL_MULT = 1f;
    private static float SLOW_MULT = 0.4f;
    private static float SUPERSLOW_MULT = 0.3f;
    //    private float floatFactor = 2f * 0.5f;
    private float powerMult = NORMAL_MULT;


    /**
     * Initialize the Overcharged Robot,
     * the Tank Drive, and gamepad1 & gamepad2
     */
    @Override
    public void init() {
        robot = new Robot6WheelTest(this,false);
        drive = robot.getTankDrive();
        RobotLog.ii(TAG_T, "TankDrive initialized drive is null " + (drive==null));
        this.gamepad1.reset();
        this.gamepad2.reset();
        RobotLog.ii(TAG_T, "gamepad1 & gamepad2 initialized");
        telemetry.addData("Robot Status: ", "Initialized");
        telemetry.update();
    }

    /**
     * This function stops the robot
     */
    @Override
    public void stop() {
        robot.close();
    }


    /**
     * TeleOp Run Loop
     * Main loop of TeleOp where drivers operate THE robot
     */
    @Override
    public void loop() {
        //RobotLog.ii(TAG_T, "loop start");
        long timestamp = System.currentTimeMillis();
        float x1 = gamepad1.left_stick_x;
        // is reversed by default
        float y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        // is reversed by default
        float y2 = -gamepad1.right_stick_y;
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);
        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);

        float arm_y1 = -gamepad2.left_stick_y;
        float arm_y2 = -gamepad2.right_stick_y;
        arm_y1 = Button.scaleInput(arm_y1);
        arm_y2 = Button.scaleInput(arm_y2);

        if (gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timestamp)) {
            isSlow = !isSlow;
            isSuperSlow = false;
        } else if (gamepad1.left_bumper && Button.BTN_SUPERSLOW_MODE.canPress(timestamp)) {
            isSuperSlow = !isSuperSlow;
            isSlow = false;
        }

        ///LB		RB		Result
        ///=============================================================
        ///Off		Off		Normal Speed
        ///Off		On		Slow Mode
        ///On		Off		Super Slow
        ///On (1)	On (2)	Slow Mode - Turn Off Super Slow Mode
        ///On (2)	On (1)	Super Slow - - Turn Off Slow Mode
        if (isSuperSlow) {
            //RobotLog.ii(TAG_T, "drive super slow mode");
            //y1 *= SUPERSLOW_MULT;
            //y2 *= SUPERSLOW_MULT;
            powerMult = SUPERSLOW_MULT;
        } else if (isSlow) {
            //RobotLog.ii(TAG_T, "drive slow mode");
            //y1 *= SLOW_MULT;
            //y2 *= SLOW_MULT;
            powerMult = SLOW_MULT;
        } else {
            //RobotLog.ii(TAG_T, "drive normal mode");
            //y1 *= NORMAL_MULT;
            //y2 *= NORMAL_MULT;
            powerMult = NORMAL_MULT;
        }

        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        arm_y1 = Range.clip(arm_y1, -1f, 1f);
        arm_y2 = Range.clip(arm_y2, -1f, 1f);
        //RobotLog.ii(TAG_T, "Gamepad1: y1=" + y1 + " y2=" + y2 + " arm_y1=" + arm_y1 + " arm_y2=" + arm_y2);

        //drive.setPower(y1, y2); //Old Tank drive
        drive.setPower(x1, y1, x2, y2, powerMult);

        ///Update LEDs to display the robot's status
        //telemetry.update();
        //RobotLog.ii(TAG_T, "End loop");
    }
}
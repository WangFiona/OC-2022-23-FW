package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

/*
 * Overcharged Team #12599 Autonomous
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "autotest", group = "Test")
public class TestDev2 extends LinearOpMode {

    ///Overcharged Autonomous Robot class
    private  Robot6WheelLinear robot;

    ///Drive components

    /**
     * Autonomous opMode, entry point into the program
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // init
            robot = new Robot6WheelLinear(this);
            telemetry.addLine("initialized");
            telemetry.update();
            run();
        } catch (Exception e){
            telemetry.addLine("init failed" + e);
            telemetry.update();
            throw e;
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Autonomous run function
     * This is the main function that performs all the actions at once
     * @throws InterruptedException
     */
    public void run() throws InterruptedException {
        WaitLinear lp = new WaitLinear(this);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
/*
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())
        {
            sleep(50);
            idle();
        }

 */
        //telemetry.addData("imu calib status", robot.gyroSensor.getCalibrationStatus().toString());

        telemetry.addData("Waiting", "Autonomous");
        telemetry.update();
        waitForStart();

        //The time when the autonomous run started i.e. the start/stop button was pressed
        long startTime = System.currentTimeMillis();

        //return immediately if stop has been pressed instead of play
        if (isStopRequested()) {
            return;
        }

        ///If the user presses stop, waitForStart() will return, only run if the start button is pressed (not stop).
        if (opModeIsActive()) {
            telemetry.addData("Autonomous", " done in " + (System.currentTimeMillis() - startTime) + " milliseconds");
            telemetry.update();
            //wait a bit longer than the 30 seconds just in case the start was accidentally pressed earlier
            lp.waitMillis(32000 - (int) (System.currentTimeMillis() - startTime));
        }
        ///End
    }
}
package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RotationAxis;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.RobotTankMecanumLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

/*
 * Overcharged Team #12599 Autonomous
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoTestTurn", group = "Test")
public class AutonomousTestTurn extends LinearOpMode {
    ///Overcharged Autonomous Robot class
    private RobotTankMecanumLinear robot;
    ///Overcharged Swerve Drive class
    private TankDriveLinear drive;

    float power = .80f;

    /**
     * Autonomous opMode, entry point into the program
     */
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // init
            robot = new RobotTankMecanumLinear(this);
            RobotLog.ii(TAG_A, "Robot6WheelLinear initialized");
            drive = robot.getTankDriveLinear();
            RobotLog.ii(TAG_A, "TankDriveLinear initialized");
            drive.odometryLocalization.resetEncoder();
            run();
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

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())
        {
            sleep(50);
            idle();
        }
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
            final double turnMediumPower = 0.4;
            boolean isRed = true;
            //the first curve angle
            double turnAngleR_FIRST = 90;
            double turnAngleB_FIRST = -90;
            double turnAngle = (isRed ? turnAngleR_FIRST : turnAngleB_FIRST);
            float distanceR_before_collecting = 16f; //14.2
//            drive.odometryLocalization.showValues();
//            drive.odometryLocalization.resetEncoder();
            //drive.turnUsingPID(turnAngle, turnMediumPower, isRed ? RotationAxis.LEFT : RotationAxis.RIGHT);
            drive.turnUsingPID(turnAngle, turnMediumPower, 0,0,isRed ? RotationAxis.LEFT : RotationAxis.RIGHT,true,0);
            drive.odometryLocalization.showValues();
            drive.odometryLocalization.resetEncoder();
            drive.stop();
//            telemetry.addData("Inches", drive.encoder.getDistanceToString());
//            telemetry.addData("Encoder", drive.encoder.getCurrentPositionString());
//            telemetry.addData("Drift", drive.encoder.getDrift());
            telemetry.addData("Autonomous", " done in " + (System.currentTimeMillis() - startTime) + " milliseconds");
            telemetry.update();
            //wait a bit longer than the 30 seconds just in case the start was accidentally pressed earlier
            lp.waitMillis(32000 - (int) (System.currentTimeMillis() - startTime));
        }
        ///End
    }
}
package overcharged.test;

// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.path.PathPlanner;
import overcharged.linear.components.Robot6WheelTestLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;
@Disabled
@Autonomous(name="Test Smooth Path", group="Test")
public class TestSmoothPath extends LinearOpMode
{
    ///Overcharged Autonomous Robot class
    private Robot6WheelTestLinear robot;
    ///Overcharged Swerve Drive class
    private TankDriveLinear drive;

    float power = .30f;
    boolean aButton, bButton;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // init
        robot = new Robot6WheelTestLinear(this);
        RobotLog.ii(TAG_A, "Robot6WheelLinear initialized");
        drive = robot.getTankDriveLinear();

        WaitLinear lp = new WaitLinear(this);
        SelectLinear sl = new SelectLinear(this);
        ///Make the alliance selection (Red/Blue)
        boolean isRed = sl.selectAlliance();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.
        waitForStart();

        ///Move in a square box
        telemetry.addData("Mode", "running");
        telemetry.update();

        //create waypoint path
        double[][] waypoints = new double[][]{
                {0, 1},
                {0.25, 2},
                {0.5, 3},
                {0.75, 5},
                {.75, 7},
                {1, 8},
                {2, 8},
                {3, 9},
                {4, 11},
                {5, 12},
                {6, 13},
                {7, 13},
                {8, 13},
                {9, 13},
                {10, 13},
                {11, 13},
                {12, 13},
                {13, 13}
        };
        //waypoints in feet
        waypoints = new double[][]{
                {0, 0},
                {0, 1},
                {0, 2},
                {1, 2},
                {2, 2},
                {3, 2},
                {4, 2}
        };

        double totalTime = 20; //max seconds we want to drive the path
        double timeStep = 0.1; //period of control loop on Rio, seconds
        double robotTrackWidth = 1.167; //distance between left and right wheels, feet

        PathPlanner path = new PathPlanner(waypoints);
        path.calculate(totalTime, timeStep, robotTrackWidth);

        for(double[] p : path.smoothCenterVelocity) {
            if (isRed) {
                drive.setPower(p[1], p[0]);
                telemetry.addData("Power", "Left=" + p[1] + " Right=" + p[0]);
            } else {
                drive.setPower(p[0], p[1]);
                telemetry.addData("Power", "Left=" + p[0] + " Right=" + p[1]);
            }
            telemetry.update();
        }

        // turn the motors off.
        drive.stop();
        telemetry.addData("Mode", "stop");
        telemetry.update();
		lp.waitMillis(5000);
        ///End
    }
}

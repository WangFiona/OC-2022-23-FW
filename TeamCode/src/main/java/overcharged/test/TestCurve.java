package overcharged.test;

// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.RotationAxis;
import overcharged.linear.components.Robot6WheelTestLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;
@Disabled
@Autonomous(name="Test Curve", group="Test")
public class TestCurve extends LinearOpMode
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

        //drive.moveToEncoderInch(12,.30f, 3000, false, false, true, null);
        //drive.turn(89.0f,.30f, 3000, false);
        RobotLog.ii(TAG_A, "Moving forwards");
        drive.moveToEncoderInchUsingPID(17, power, 3000, true);
        drive.turnUsingPID(-84,.20, RotationAxis.CENTER);
        drive.moveToEncoderInchUsingPID(-17, power, 3000, true);
        lp.waitMillis(2000);
        RobotLog.ii(TAG_A, "Moving backwards");
        drive.moveToEncoderInchUsingPID(25, power, 3000, true);

//        drive.turnUsingPID(84,.20);
//        //drive.moveToEncoderInch(2,.30f, 3000, false, false, true, null);
//        drive.moveToEncoderInchUsingPID(2, .power, 3000, true);
//        drive.turnUsingPID(84,.20);
//        //drive.moveToEncoderInch(12,.30f, 3000, false, false, true, null);
//        drive.moveToEncoderInchUsingPID(12, power, 3000, true);
//        telemetry.addData("Travel", "stopping");

        //drive.curelTravel(10, 30f, power, 3000, false, false, true, null);

        // turn the motors off.
        drive.stop();
        telemetry.addData("Mode", "stop");
        telemetry.update();
		lp.waitMillis(5000);
        ///End
    }
}

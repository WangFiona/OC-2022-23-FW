package overcharged.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.oldRobotMecanum;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.WaitLinear;

//@Disabled
@Autonomous(name="mecanumTurn")
public class mecanumTurn extends LinearOpMode {

    oldRobotMecanum robot;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new oldRobotMecanum(this, true, true);
        WaitLinear lp = new WaitLinear(this);
        drive = new SampleMecanumDrive(hardwareMap);

        if (isStopRequested()) {
            return;
        }
        waitForStart();

        if (opModeIsActive()) {
            drive.turn(Math.toRadians(17));
        }
    }
}
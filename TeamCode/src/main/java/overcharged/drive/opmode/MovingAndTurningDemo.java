package overcharged.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.drive.SampleMecanumDrive;
import overcharged.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name="fwdandback")
public class MovingAndTurningDemo extends LinearOpMode {
    Trajectory lastTrajectory = null;
    public static Vector2d shippingHub = new Vector2d(45, 25);

    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        Trajectory first = drive.trajectoryBuilder(new Pose2d()).
                forward(shippingHub.getX()).
                build();
        Trajectory second = drive.trajectoryBuilder(first.end()).
                strafeLeft(shippingHub.getY()).
                build();

        Trajectory third = drive.trajectoryBuilder(second.end()).
                strafeRight(shippingHub.getY()).
                build();

        Trajectory fourth = drive.trajectoryBuilder(third.end()).
                back(shippingHub.getX()).
                build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            drive.followTrajectory(first);
            sleep(1000);
            drive.followTrajectory(second);
            sleep(1000);

            drive.followTrajectory(third);
            sleep(1000);

            drive.followTrajectory(fourth);
            sleep(1000);

        }

    }


}

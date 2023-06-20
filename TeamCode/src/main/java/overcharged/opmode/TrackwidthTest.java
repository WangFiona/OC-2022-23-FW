package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import overcharged.components.Drive;
import overcharged.components.Robot6Wheel;
import overcharged.components.TankDrive;

@Disabled
@TeleOp(name="TrackwidthTest", group="Game")
public class TrackwidthTest extends OpMode {

    private Robot6Wheel robot;
    private TankDrive drive;

    @Override
    public void init() {
        robot = new Robot6Wheel(this,false);
        drive = robot.getTankDrive();

        robot.driveLeftFront.resetPosition();
        robot.driveLeftBack.resetPosition();
        robot.driveRightFront.resetPosition();
        robot.driveRightBack.resetPosition();
    }

    @Override
    public void loop() {
        if(gamepad1.right_stick_x != 0){
            robot.driveLeftFront.setPower(0.6*gamepad1.right_stick_x);
            robot.driveLeftBack.setPower(0.6*gamepad1.right_stick_x);
            robot.driveRightFront.setPower(-0.6*gamepad1.right_stick_x);
            robot.driveRightBack.setPower(-0.6*gamepad1.right_stick_x);
        } else {
            drive.setPower(0);
        }

        telemetry.addData("Calculated Trackwidth over 20 rotations is", calculateTrackwidth(20));
        telemetry.addData("Tick to inch of old", 674.0211839941768/Drive.AMO20_ENCODER_TICK_PER_INCH);
        telemetry.update();
    }

    public double calculateTrackwidth(int rot){
        double left = robot.driveLeftFront.getCurrentPosition();
        double right = robot.driveRightFront.getCurrentPosition();

        double leftTrackwidth = Math.abs(left/Math.PI)/rot;
        double rightTrackwidth = Math.abs(right/Math.PI)/rot;

        return (leftTrackwidth+rightTrackwidth)/2;
    }
}

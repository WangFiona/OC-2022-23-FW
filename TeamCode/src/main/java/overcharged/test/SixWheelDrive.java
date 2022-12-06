package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.OcServo;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.components.RotationAxis;
import overcharged.linear.components.MoveAction;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.TankDriveLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

@Disabled
@Autonomous(name = "SixWheelDrive", group = "Game")
public class SixWheelDrive extends LinearOpMode {

    DcMotor driveLF;
    DcMotor driveLB;
    DcMotor driveRF;
    DcMotor driveRB;

    private TankDriveLinear drive;

    ///Constants
    public final float intake_power = 0.7f;
    final float movepower = .70f;

    private boolean isLeft = true;
    private RingPosition ringPosition = RingPosition.A;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;

    /**
     * Autonomous opMode, entry point into the program
     */
    @Override
    public void runOpMode() throws InterruptedException
    {

        driveLF = hardwareMap.dcMotor.get("driveLeftFront");
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        driveLB = hardwareMap.dcMotor.get("driveLeftBack");
        driveRF = hardwareMap.dcMotor.get("driveRightFront");
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        driveRB = hardwareMap.dcMotor.get("driveRightBack");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power.

        driveLB.setPower(0.25);
        driveLF.setPower(0.25);
        driveRF.setPower(0.25);
        driveRB.setPower(0.25);

        sleep(2000);

        driveLF.setPower(0.0);
        driveLB.setPower(0.0);
        driveRB.setPower(0.0);
        driveRF.setPower(0.0);

        sleep(2000);

        driveLB.setPower(-0.25);
        driveLF.setPower(-0.25);
        driveRF.setPower(0.25);
        driveRB.setPower(0.25);

        sleep(2000);

        driveLF.setPower(0.0);
        driveLB.setPower(0.0);
        driveRB.setPower(0.0);
        driveRF.setPower(0.0);

    }


}


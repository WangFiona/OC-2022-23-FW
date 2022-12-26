package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.DuckDetector;
import overcharged.components.MecanumDrive;
import overcharged.components.RobotMecanum;
import overcharged.components.SignalColors;
import overcharged.components.hSlides;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.SignalConePipeLine;

@Autonomous(name="parkingAuto")
public class parkingAuto extends LinearOpMode {

    private RobotMecanum robot;
    private MecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    long currentTime;
    SignalConePipeLine detector;
    OpenCvWebcam webcam;
    private SignalColors signalColors = SignalColors.Red;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    double drivePower = -0.4f;
    float turretPower = 0.6f;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot = new RobotMecanum(this, true, false);
            drive = robot.getDrive();
            WaitLinear lp = new WaitLinear(this);
            initCamera();

            robot.clawGrab();

            this.detector = new SignalConePipeLine();
            //this.detector.useDefaults();
            webcam.setPipeline(detector);
            detector.isLeft(false);

            try {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch(Exception e) {
                try {
                    this.detector = new SignalConePipeLine();
                    //this.detector.useDefaults();
                    webcam.setPipeline(detector);
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                } catch (Exception f) {
                    telemetry.addLine("Error");
                    telemetry.update();
                    signalColors = SignalColors.Red;
                }
            }

            /*while (!isStopRequested() && robot.gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro Status", "Calibrating");
                telemetry.update();
                sleep(50);
                idle();
            }*/
            //telemetry.addData("Gyro Status", "Calibrated");
            telemetry.update();

            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                signalColors = detector.getSignalColors();
                currentTime = System.currentTimeMillis();
            }

            detector.reset();
            telemetry.addData("Signal Color", signalColors);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();
            //drive.resetAngle();
            if (opModeIsActive()) {
                time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                    signalColors = detector.getSignalColors();
                    currentTime = System.currentTimeMillis();
                }

                detector.reset();
                telemetry.addData("Signal Color", signalColors);
                telemetry.update();

                webcam.stopStreaming();
                webcam.closeCameraDevice();

                //function here
                park(lp);
            }

        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Camera Initialization Function
     */
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        //webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
        webcam.openCameraDevice();
    }

    public void park(WaitLinear lp) throws InterruptedException {
        robot.vSlides.moveTo2();
        lp.waitMillis(500);
        telemetry.addData("init", hSlides.INIT);
        telemetry.update();
        robot.hSlides.setPosition(95f);
        robot.turret.moveTo(36, turretPower);
        lp.waitMillis(1500);
        robot.clawOpen();
        lp.waitMillis(500);
        reset(lp);
        //lp.waitMillis(1400);
        if(signalColors == SignalColors.Red){
            robot.drive.setStrafePower((float)(drivePower));
            lp.waitMillis(2000);
            robot.drive.setPower(drivePower);
            lp.waitMillis(1600);
        } else if(signalColors == SignalColors.Green){
            robot.drive.setStrafePower((float)(-drivePower));
            lp.waitMillis(2500);
            robot.drive.setPower(drivePower);
            lp.waitMillis(1600);
        } else{
            robot.drive.setPower(drivePower);
            lp.waitMillis(1700);
        }

        robot.turret.moveTo(0, 0.9f);
    }
    public void reset(WaitLinear lp) throws InterruptedException {
        robot.hSlides.setPosition(hSlides.IN);
        robot.turret.moveTo(0, turretPower);
        lp.waitMillis(100);
        slideBottom(lp);
        lp.waitMillis(1000);
        robot.clawOpen();
    }

    public void slideBottom(WaitLinear lp) {
        if (!robot.vSlides.switchSlideDown.isTouch() && robot.vSlides.getCurrentPosition() > robot.vSlides.start){//!robot.vSlides.slideReachedBottom()){
            robot.vSlides.moveToBottom();
        } else {
            robot.vSlides.forcestop();
            robot.vSlides.reset(robot.vSlides.slideLeft);
            robot.vSlides.reset(robot.vSlides.slideRight);
            robot.clawOpen();
        }
    }
}

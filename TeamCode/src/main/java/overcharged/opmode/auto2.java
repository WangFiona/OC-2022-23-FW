package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import overcharged.components.DuckDetector;
import overcharged.components.RobotMecanum;
import overcharged.components.SignalColors;
import overcharged.components.hSlides;
import overcharged.drive.DriveConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
//import overcharged.opmode.mSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.SignalConePipeLine;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto2")
public class auto2 extends LinearOpMode {

    private RobotMecanum robot;
    SampleMecanumDrive drive;
    SelectLinear sl = new SelectLinear(this);
    long startTime;
    long currentTime;
    private SignalColors signalColors = SignalColors.Red;
    SignalConePipeLine detector;
    OpenCvWebcam webcam;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    boolean Left = true;
    double turretPower = 0.6;
    double y3 = -60;
    int cone1 = 25;//30;
    int interval = 67;
    boolean grabbed = true;
    //int hSlidesReset = 150;

    TrajectorySequence toSquare3, toLine, toLine2, to1, to2, to3, toScore, correct, correct2, to2p2, score, toR1, toR3;
    Pose2d start = new Pose2d();

    Vector2d line, s3, p1, p2, p3, line2, p2p2, scorePos;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    MultipleTelemetry telems;


    @Override
    public void runOpMode() throws InterruptedException {
        try {
            telems = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);


            robot = new RobotMecanum(this, true, true);
            WaitLinear lp = new WaitLinear(this);
            initCamera(); // initializes camera and sets up pipeline for team shipping element detection

            // when we start, we want the outtake mechanism in a predictable position
            robot.clawGrab();

            drive = new SampleMecanumDrive(hardwareMap);

            Left = sl.selectPosition();

            double xVal = (Left? 50: 51);
            double yVal = (Left? 11: -9); //-6
            double yVal2 = (Left? 8.5: -6); //-6
            line = new Vector2d(xVal, yVal);
            line2 = new Vector2d(xVal, yVal2);
            scorePos = new Vector2d(xVal, (Left? -15: 15));
            s3 = new Vector2d(xVal,0);
            p1 = new Vector2d(xVal-3, (Left? 23 : 25));
            p2 = new Vector2d(xVal-7, (Left? 0 : 3)); //xVal-3
            p2p2 = new Vector2d(xVal-8, (Left? 0 : 3));
            p3 = new Vector2d(xVal-3, -24);

            toSquare3 = drive.trajectorySequenceBuilder(start)
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(s3)
                    .addSpatialMarker(new Vector2d(1,0), () -> {
                        robot.turret.moveTo((Left? -34 : 48), turretPower);
                        robot.vSlides.moveTo(1900);
                        robot.hSlides.setPosition(Left? 128f : 128f);})
                    .build();

            toLine = drive.trajectorySequenceBuilder(toSquare3.end())
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .strafeTo(line)
                    .build();

            correct = drive.trajectorySequenceBuilder(new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), 0))
                    .lineTo(line)
                    .build();

            correct2 = drive.trajectorySequenceBuilder(new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), 0))
                    .lineTo(line2)
                    .build();

            score = drive.trajectorySequenceBuilder(toLine.end())
                    .lineTo(scorePos)
                    .build();

            to1 = drive.trajectorySequenceBuilder(toLine.end())
                    .strafeTo(p1)
                    .build();

            toR1 = drive.trajectorySequenceBuilder(toLine.end())
                    .strafeTo(p1)
                    .addSpatialMarker(new Vector2d(xVal-2,4), () -> {
                        lowerSlidesThread(lp, 1);})
                    .build();

            to2 = drive.trajectorySequenceBuilder(toLine.end())
                    .strafeTo(p2)
                    //.addSpatialMarker(new Vector2d(xVal-3,2), () -> {
                        //lowerSlidesThread(lp, 1);})
                    .build();


            to2p2 = drive.trajectorySequenceBuilder(to2.end())
                    .lineTo(p2p2)
                    .build();

            to3 = drive.trajectorySequenceBuilder(toLine.end())
                    .strafeTo(p3)
                    .addSpatialMarker(new Vector2d(-2,-4), () -> {
                        lowerSlidesThread(lp, 1);})
                    .build();

            toR3 = drive.trajectorySequenceBuilder(toLine.end())
                    .strafeTo(p3)
                    //.addSpatialMarker(new Vector2d(xVal-3,2), () -> {
                    //lowerSlidesThread(lp, 1);})
                    .build();

            this.detector = new SignalConePipeLine();
            //this.detector.useDefaults();
            webcam.setPipeline(detector);

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

            telemetry.update();

            long time1 = System.currentTimeMillis();
            currentTime = System.currentTimeMillis();
            while (currentTime - time1 < DuckDetector.DETECTION_WAIT_TIME) {
                signalColors = detector.getSignalColors();
                currentTime = System.currentTimeMillis();
            }

            //detector.reset();
            telemetry.addData("Signal Color", signalColors);
            telemetry.addData("Left?", Left);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();

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

                AutoBody(lp,Left);
            }

        } catch (Exception e) {
            RobotLog.e("Overcharged: Autonomous Failed: ", e.getMessage());
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
    }

    public void AutoBody(WaitLinear lp, boolean Left) throws InterruptedException {
        RobotLog.ii(TAG_SL, "started");
        startTime = System.currentTimeMillis();
        drive.setPoseEstimate(start);
        double startEL = drive.leftRear.getCurrentPosition();
        double startEM = drive.rightRear.getCurrentPosition();
        double startER = drive.rightFront.getCurrentPosition();

        robot.hSlides.setPosition(154f);
        drive.followTrajectorySequence(toSquare3);
        telemetry.addData("L start", startEL);
        telemetry.addData("L end", drive.leftRear.getCurrentPosition());
        telemetry.addData("R start", startER);
        telemetry.addData("R end", drive.rightFront.getCurrentPosition());
        telemetry.addData("M start", startEM);
        telemetry.addData("M end", drive.rightRear.getCurrentPosition());
        telemetry.addData("location", drive.getPoseEstimate().getX());
        telemetry.addData("location", drive.getPoseEstimate().getY());
        telemetry.update();

        //lp.waitMillis(400);
        /*robot.turret.moveTo((Left? -34 : 49), turretPower);
        robot.vSlides.moveTo(1900);
        lp.waitMillis(1200);
        robot.hSlides.setPosition(Left? 118f : 105f);
        //lp.waitMillis(500);
        //robot.vSlides.moveTo(1700);
        lp.waitMillis(500);*/

        robot.hSlides.setPosition(Left? 115f : 95f);
        lp.waitMillis(500);
        robot.vSlides.moveTo(1780);
        lp.waitMillis(150);
        robot.claw.setAutoOpen();
        //lp.waitMillis(200);

        reset90(lp, Left, 5, false);
        drive.followTrajectorySequence(toLine);

        /*long forwardTime = System.currentTimeMillis();
        while(robot.sensorL.getRawLightDetected() < 700 && opModeIsActive() && System.currentTimeMillis() - forwardTime < 300){
            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
        }
        drive.setMotorPowers(0, 0, 0, 0);*/

        long parkTime = 4500; //5500
        if(signalColors == SignalColors.Red)
            parkTime += Left? 1000 : 1000;
        else if(signalColors == SignalColors.Blue)
            parkTime += 1000;
        else
            parkTime += Left? 1000 : 1000;

        int cLevel = 5;
        boolean firstTime = true;
        while(System.currentTimeMillis() - startTime < (30000-parkTime) && opModeIsActive() && cLevel >= 1) {
            if(!grabbed)
                cLevel++;

            if(firstTime)
                firstTime = false;
            else
                reset90(lp, Left, cLevel, true);
            RobotLog.ii(TAG_SL, "cLevel: " + cLevel);

            cLevel--;
            grabbed = false;

            float hSlidesOut = 85f;
            while (robot.sensorF.getDistance(DistanceUnit.CM) > 1.5 && hSlidesOut >= hSlides.MIN){//hSlidesOut >= hSlides.MIN+10) {
                hSlidesOut -= 3;
                robot.hSlides.setPosition(hSlidesOut);
                lp.waitMillis(40);
            }

            RobotLog.ii(TAG_SL, "sensorF distance: " + robot.sensorF.getDistance(DistanceUnit.CM) + "hSlidesOut: "  + hSlidesOut + "vSlides: currentPos R: " + robot.vSlides.slideRight.getCurrentPosition() +
                    "currentPos L: " + robot.vSlides.slideLeft.getCurrentPosition());
            //robot.claw.setAutoGrab();
            robot.clawGrab();
            lp.waitMillis(350);
            robot.hSlides.setPosition(hSlidesOut+10);
            //robot.hSlides.setPosition(110f);
            robot.vSlides.moveTo4();
            lp.waitMillis(200);

            if(robot.sensorF.getDistance(DistanceUnit.CM) < 10)
                grabbed = true;

            robot.hSlides.setPosition(110f);
            robot.turret.moveTo((Left? -45.5 : 56), turretPower); //-46
            lp.waitMillis(1200);

            if(robot.sensorF.getDistance(DistanceUnit.CM) < 10 && grabbed) {
                robot.hSlides.setPosition(hSlides.MIN);
                drive.followTrajectorySequence(correct2);
                //lp.waitMillis(800);
                robot.vSlides.moveTo(1780);
                lp.waitMillis(150);
            }
            robot.claw.setAutoOpen();
        }

        lp.waitMillis(200);
        robot.hSlides.setPosition(hSlides.MAX);
        //lp.waitMillis(200);
        robot.turret.moveTo(0, turretPower);
        robot.vSlides.moveTo2();
        //lowerSlidesThread(lp,1);
        //lp.waitMillis(200);
        //robot.clawOpen();

        drive.followTrajectorySequence(correct);

        if(signalColors == SignalColors.Red){
            if(Left) {
                drive.followTrajectorySequence(to3);
            } else {
                lowerSlidesThread(lp, 1);
                drive.followTrajectorySequence(toR3);
            }
        } else if(signalColors == SignalColors.Green){
            if(Left) {
                lowerSlidesThread(lp, 1);
                drive.followTrajectorySequence(to1);
            } else {
                drive.followTrajectorySequence(toR1);
            }
        } else{ //blue
            lowerSlidesThread(lp, 1);
            drive.followTrajectorySequence(to2);
        }

        //lowerSlidesThread(lp, 1);
        robot.turret.moveTo(0, 0.9);
    }

    public void reset90(WaitLinear lp, boolean Left, int newL, boolean wait) throws InterruptedException {
        if(!wait){
            robot.hSlides.setPosition(128f);
            lp.waitMillis(500);
        } else {
            robot.hSlides.setPosition((Left ? 85f : 90));
            //if(wait)
            lp.waitMillis(250);
        }

        robot.turret.moveTo((Left? 86 : -79.5), turretPower); //87
        if(!wait) {
            robot.hSlides.setPosition((Left ? 85f : 90));
            //lp.waitMillis(200);
        }

        robot.vSlides.moveTo(cone1+(interval*newL));
        if(wait) {
            drive.followTrajectorySequence(correct);
            lp.waitMillis(200);
        }
    }

    public void lowerSlidesThread(WaitLinear lp, int level) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(false, lp, this, robot, signalColors, cone1, interval, level);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}

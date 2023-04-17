package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
//import overcharged.opmode.mSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.SignalConePipeLine;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto10RedAtHome")
public class auto10RedAtHome extends LinearOpMode {

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
    float turretPower = 0.9f;
    double y3 = -60;
    int cone1 = -38;//22;//5;
    int interval = 55;//67
    boolean grabbed = true;
    double resetAngle = 78;
    int notGrab = 0;
    float dumpLengthL = 90f;
    float dumpLength2L = 114f;//118f;
    float dumpLengthR = 90f;
    float dumpLength2R = 122f;
    float close = 12f;
    float far = 15f;
    int Length = 0;
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
            //robot.clawGrab();

            drive = new SampleMecanumDrive(hardwareMap);

            Left = sl.selectPosition();

            Length = sl.selectLength();
            if(Length == 2){
                dumpLengthL -= close;
                dumpLength2L -= close;
                dumpLengthL -= close;
                dumpLength2R -= close;
            } else if(Length == 1){
                dumpLengthL += far;
                dumpLength2L += far;
                dumpLengthR += far;
                dumpLength2R += far;
            }

            if(!Left)
                resetAngle = 79;

            double xVal = (Left? 50: 51);
            double yVal = (Left? 11: -9); //-6
            double yVal2 = (Left? 6: -6); //8.5
            line = new Vector2d(xVal, yVal);
            line2 = new Vector2d(xVal, yVal2);
            scorePos = new Vector2d(xVal, (Left? -15: 15));
            s3 = new Vector2d(xVal,0);
            p1 = new Vector2d((Left? xVal-1 : xVal+1), (Left? 27 : 27));
            p2 = new Vector2d(xVal-7, (Left? -2 : 3)); //xVal-3
            p2p2 = new Vector2d(xVal-8, (Left? 0 : 3));
            p3 = new Vector2d(xVal-(Left? -4 : 4), -25);

            toSquare3 = drive.trajectorySequenceBuilder(start)
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineTo(s3)
                    /*.addSpatialMarker(new Vector2d((Left? 1 : 1),0), () -> {
                        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);
                    })*/
                    .addSpatialMarker(new Vector2d((Left? 5 : 5),0), () -> {
                        robot.alignerOut();
                        if(!Left){robot.aligner.setRight();}
                    })
                    .addSpatialMarker(new Vector2d((Left? 10 : 10),0), () -> {
                        robot.vSlides.moveTo(1410);
                        //robot.alignerOut();
                        //if(!Left){robot.aligner.setRight();}
                    })
                    .addSpatialMarker(new Vector2d((Left? 27 : 27), 0), () -> {
                        robot.turret.moveTo((Left? 71 : -73), 1f);//45
                    })
                    .addSpatialMarker(new Vector2d((Left? 49.9 : 50.9),0), () -> {
                        robot.vSlides.moveTo(1200);
                        robot.turret.setPower(0);
                    })
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
                    .addSpatialMarker(new Vector2d(line2.getX(),Left ? 8 : -8), () -> {
                        robot.hSlides.setPosition(Left ? dumpLength2L : dumpLength2R);//164f : 142f);//157f : 156f);
                    })
                    .build();

            score = drive.trajectorySequenceBuilder(toLine.end())
                    .lineTo(scorePos)
                    .build();

            to1 = drive.trajectorySequenceBuilder(toLine.end())
                    .lineToLinearHeading(new Pose2d(p1, 0))//Left? Math.toRadians(-90) : Math.toRadians(90)))
                    .addSpatialMarker(new Vector2d(xVal-1,15), () -> {
                        lowerSlidesThread(lp, 1);})
                    .build();

            toR1 = drive.trajectorySequenceBuilder(toLine.end())
                    .lineToLinearHeading(new Pose2d(p1, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d((Left? xVal-1 : xVal+1), (Left? 27 : 23)), () -> {
                        if(robot.vSlides.switchSlideDown.isTouch())
                            robot.claw.setPosition(135f);
                    })
                    .build();

            to2 = drive.trajectorySequenceBuilder(toLine.end())
                    //.lineToLinearHeading(new Pose2d(p2, 0))
                    .splineToLinearHeading(new Pose2d(p2, 0), 0)
                    //.addSpatialMarker(new Vector2d(xVal-3,2), () -> {
                    //lowerSlidesThread(lp, 1);})
                    .build();

            to3 = drive.trajectorySequenceBuilder(toLine.end())
                    .lineToLinearHeading(new Pose2d(p3, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(xVal-(Left? -4 : 4), -18), () -> {
                        if(robot.vSlides.switchSlideDown.isTouch())
                            robot.claw.setPosition(135f);
                    })
                    .build();

            toR3 = drive.trajectorySequenceBuilder(toLine.end())
                    .lineToLinearHeading(new Pose2d(p3, 0))
                    //.addSpatialMarker(new Vector2d(xVal-3,2), () -> {
                    //lowerSlidesThread(lp, 1);})
                    .build();

            this.detector = new SignalConePipeLine();
            //this.detector.useDefaults();
            webcam.setPipeline(detector);
            detector.isLeft(Left);

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
            telemetry.addData("Length", Length);
            telemetry.addData("Dump LengthL", dumpLengthL);
            telemetry.addData("Dump Length2L", dumpLength2L);
            telemetry.addData("Dump LengthR", dumpLengthR);
            telemetry.addData("Dump Length2R", dumpLength2R);
            telemetry.update();

            if (isStopRequested()) {
                return;
            }

            waitForStart();

            if (opModeIsActive()) {
                startTime = System.currentTimeMillis();
                time1 = System.currentTimeMillis();
                currentTime = System.currentTimeMillis();
                robot.clawGrab();
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
        drive.setPoseEstimate(start);
        double startEL = drive.leftRear.getCurrentPosition();
        double startEM = drive.rightRear.getCurrentPosition();
        double startER = drive.rightFront.getCurrentPosition();

        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);
        drive.followTrajectorySequence(toSquare3);
        robot.vSlides.moveTo(1200);
        //lp.waitMillis(150);

        robot.claw.setAutoOpen();
        robot.aligner.setMiddle();

        reset90(lp, Left, 5, false);
        //drive.followTrajectorySequence(toLine);

        long parkTime = 4500; //5500
        if(signalColors == SignalColors.Red)
            parkTime += Left? 1000 : 1000;
        else if(signalColors == SignalColors.Blue)
            parkTime += 1000;
        else
            parkTime += Left? 1000 : 1000;

        int cLevel = 5;
        boolean firstTime = true;
        int numGrabbed = 0;
        while(System.currentTimeMillis() - startTime < (30000-parkTime) && opModeIsActive() && cLevel >= 1 && notGrab < 2 && numGrabbed<5) {
            numGrabbed++;
            if(!grabbed)
                cLevel++;

            if(firstTime)
                firstTime = false;
            else
                reset90(lp, Left, cLevel, true);
            RobotLog.ii(TAG_SL, "cLevel: " + cLevel);
            robot.turret.turret.setTargetPositionPIDFCoefficients(1.4,0,0,0);

            cLevel--;
            grabbed = false;

            float hSlidesOut = robot.hSlides.getPos();//153f;
            while (robot.sensorF.getDistance(DistanceUnit.CM) > 4 && hSlidesOut <= 142f){//hSlidesOut >= hSlides.MIN+10) {
                hSlidesOut += 3;
                robot.hSlides.setPosition(hSlidesOut);
                lp.waitMillis(20);
            }

            RobotLog.ii(TAG_SL, "sensorF distance: " + robot.sensorF.getDistance(DistanceUnit.CM) + "hSlidesOut: "  + hSlidesOut + "vSlides: currentPos R: " + robot.vSlides.slideRight.getCurrentPosition() +
                    "currentPos L: " + robot.vSlides.slideLeft.getCurrentPosition());
            robot.clawGrab();
            lp.waitMillis(400);
            robot.hSlides.setPosition((hSlidesOut-10));
            robot.vSlides.moveTo(800);//moveTo4();
            lp.waitMillis(200);
            robot.hSlides.setPosition(hSlides.IN);


            robot.turret.moveTo((Left? 74 : -76), turretPower); //-46
            lp.waitMillis(Left? 600 : 600);//950
            robot.vSlides.moveTo(1400);//moveTo4();
            robot.alignerOut();
            if(!Left){robot.aligner.setRight();}
            //robot.hSlides.setPosition(Left? dumpLength2L : dumpLength2R);//164f : 142f);//157f : 156f);
            drive.followTrajectorySequence(correct2);
            //lp.waitMillis(Left? 80 : 50);//950

            if(robot.sensorF.getDistance(DistanceUnit.CM) < 3) {
                grabbed = true;
            } else
                notGrab++;
            telemetry.addData("notgrab ", notGrab);
            telemetry.update();
            RobotLog.ii(TAG_SL, "isGrabbed ", grabbed + "distance " + robot.sensorF.getDistance(DistanceUnit.CM));

            if(robot.sensorF.getDistance(DistanceUnit.CM) < 10 && grabbed) {
                //robot.hSlides.setPosition(Left? 183f : 179f);
                robot.vSlides.moveTo(1200);
                robot.turret.setPower(0);
                lp.waitMillis(150);
            }else
                lp.waitMillis(300);

            robot.alignerInit();
            robot.claw.setAutoOpen();
            robot.aligner.setMiddle();
        }

        robot.aligner.setLeft();
        robot.turret.turret.setTargetPositionPIDFCoefficients(7,0,0,0);

        //lp.waitMillis(300);
        robot.hSlides.setPosition(hSlides.IN);
        lp.waitMillis(350);
        robot.clawGrab();
        robot.turret.moveTo(0, turretPower);
        robot.vSlides.moveToT();

        //drive.followTrajectorySequence(correct);
        //lp.waitMillis(150);

        lowerSlidesThread(lp, 1);
        if(signalColors == SignalColors.Red){
            if(Left) {
                drive.followTrajectorySequence(to3);
            } else {
                drive.followTrajectorySequence(toR3);
            }
        } else if(signalColors == SignalColors.Green){
            if(Left) {
                drive.followTrajectorySequence(to1);
            } else {
                drive.followTrajectorySequence(toR1);
            }
        } else{ //blue
            drive.followTrajectorySequence(to2);
        }

        robot.claw.setPosition(135f);
        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
        //robot.turret.moveTo(0, 0.9f);
    }

    public void reset90(WaitLinear lp, boolean Left, int newL, boolean wait) throws InterruptedException {
        robot.turret.turret.setTargetPositionPIDFCoefficients(5,0,0,0);
        //if(!wait){
        //    robot.hSlides.setPosition(69f);//80f);
        //    lp.waitMillis(500);
        //} else {
            robot.hSlides.setPosition(Left? 65f: 65f);//126f : 133f);//140f);
        robot.alignerInit();
            lp.waitMillis(250);
        //}

        robot.turret.moveTo((Left? -resetAngle : resetAngle), turretPower); //84.5, 81.4
        robot.vSlides.moveTo(cone1+(interval*newL));
        int coneLevel = cone1+(interval*newL);
        RobotLog.ii(TAG_SL, "cone level " + coneLevel);


        if(!wait) {
            lp.waitMillis(300);
            robot.aligner.setLeft();
            lp.waitMillis(200);
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 115f : 115f);//136f//(Left? 143f : 145f);//(Left ? 85f : 100f));
            drive.followTrajectorySequence(toLine);
            //lp.waitMillis(1000);
        } else {
            lp.waitMillis(300);
            robot.aligner.setLeft();
            lp.waitMillis(200);
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 104f : 104f);//140f);
            drive.followTrajectorySequence(correct);
        }
        RobotLog.ii(TAG_SL, "actual cone level " + robot.vSlides.getCurrentPosition());
        //lp.waitMillis(1000);

        telemetry.addData("reset angle ", resetAngle);
        telemetry.update();
        while(!wait && robot.sensorF.getDistance(DistanceUnit.CM) > 7.2 && Math.abs(robot.turret.getCurrentAngle()) < 88){
            robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setPower(Left? -0.15f : 0.15f);
            telemetry.addData("current angle", robot.turret.getCurrentAngle());
            telemetry.addData("reset angle ", resetAngle);
            telemetry.addData("distance ", robot.sensorF.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.turret.setPower(0);
        if(Math.abs(robot.turret.getCurrentAngle()) > 88){
            resetAngle = 83;
            robot.turret.moveTo((Left? -resetAngle : resetAngle), turretPower); //84.5, 81.4
        } else if(!wait) {
            resetAngle = Math.abs(robot.turret.getCurrentAngle()) - (Left ? 0 : 0);
            robot.turret.moveTo((Left ? -resetAngle : resetAngle), turretPower);
        }
    }

    public void lowerSlidesThread(WaitLinear lp, int level) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.85f,false, lp, this, robot, signalColors, cone1, interval, level);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}
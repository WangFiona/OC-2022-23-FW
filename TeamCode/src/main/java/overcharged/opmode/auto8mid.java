package overcharged.opmode;


import static overcharged.config.RobotConstants.TAG_SL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name="auto8mid")
public class auto8mid extends LinearOpMode {

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
    float turretPower = 1f;
    double y3 = -60;
    int cone1 = -38;//22;//5;
    int interval = 55;//67
    boolean grabbed = true;
    double resetAngle = -13;
    int offset = 60;
    float dumpLengthL = 43f;//19f;
    float dumpLength2L = 107f;//84f;
    float dumpLengthR = 43f;//19f;
    float dumpLength2R = 107f;//84f;
    float close = 12f;
    float far = 15f;
    int Length = 0;
    int cLevel = 5;
    //int hSlidesReset = 150;

    TrajectorySequence toSquare3, toLine, toLine2, to1, to2, to3, toScore, correct, correct2, to2p2, score, toR1, toR3;
    Pose2d start = new Pose2d();

    Vector2d line, p1, p2, p3, line2, p2p2, scorePos, pR3;
    Pose2d s3;

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
            robot.aligner.autoValue();

            Left = sl.selectPosition();
            if(!Left)
                resetAngle = 15;

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

            double xVal = (Left? 50: 51);
            double yVal = (Left? 11: -12); //-6
            double yVal2 = (Left? 8.5: -6); //-6
            line = new Vector2d(xVal, yVal);
            line2 = new Vector2d(xVal, yVal2);
            scorePos = new Vector2d(xVal, (Left? -15: 15));
            s3 = new Pose2d(xVal,0, Left? Math.toRadians(90) : Math.toRadians(-90));
            p1 = new Vector2d(xVal-4, (Left? 22 : 26));
            p2 = new Vector2d(xVal-5, (Left? -3 : 3)); //xVal-3
            p2p2 = new Vector2d(xVal-8, (Left? 0 : 3));
            p3 = new Vector2d(xVal-(Left? 2 : 2), -26);
            pR3 = new Vector2d(25, -25);

            toSquare3 = drive.trajectorySequenceBuilder(start)
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(s3)
                    .addSpatialMarker(new Vector2d((Left? 1 : 1),0), () -> {
                        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);
                    })
                    .addSpatialMarker(new Vector2d((Left? 10 : 15),0), () -> {
                        robot.vSlides.moveTo(1000);//1420);
                    })
                    .addSpatialMarker(new Vector2d((Left? 20 : 20), 0), () -> {
                        robot.turret.moveTo((Left? -175 : 175), turretPower);//, Left? false : true);//-154 : 151), turretPower);
                    })
                    .addSpatialMarker(new Vector2d((Left? 30 : 30),0), () -> {
                        robot.alignerOut();
                        if(Left){robot.aligner.setRight();}
                    })
                    .addSpatialMarker(new Vector2d((Left? 49 : 50),0), () -> {
                        robot.vSlides.moveTo(850);
                        robot.turret.setPower(0);
                    })
                    .build();

            toLine = drive.trajectorySequenceBuilder(toSquare3.end())
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(line, Left? Math.toRadians(90) : Math.toRadians(-93)))
                    .addSpatialMarker(new Vector2d(line.getX(),Left? 3 : -3), () -> {
                        robot.vSlides.moveTo(cone1 + (interval * cLevel));
                    })

            //.strafeTo(line)
                    .build();

            correct = drive.trajectorySequenceBuilder(new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(line, Left? Math.toRadians(90) : Math.toRadians(-93)))
                    .build();

            correct2 = drive.trajectorySequenceBuilder(new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(line2, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .build();

            score = drive.trajectorySequenceBuilder(toLine.end())
                    .lineTo(scorePos)
                    .build();

            to1 = drive.trajectorySequenceBuilder(toLine.end())
                    .lineToLinearHeading(new Pose2d(p1, 0))
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
                    //.addSpatialMarker(new Vector2d(-2,-4), () -> {
                    //lowerSlidesThread(lp, 1);})
                    .build();

            toR3 = drive.trajectorySequenceBuilder(correct2.end())
                    //.lineToLinearHeading(new Pose2d(line2.getX(), line2.getY()+7, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    //.lineToLinearHeading(new Pose2d(25, line2.getY()+7, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    //.lineToLinearHeading(new Pose2d(25, -22, Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .lineToLinearHeading(new Pose2d(p3, 0))
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
            telemetry.addData("Aligner Value", robot.aligner.OUT);
            /*telemetry.addData("Dump LengthL", dumpLengthL);
            telemetry.addData("Dump Length2L", dumpLength2L);
            telemetry.addData("Dump LengthR", dumpLengthR);
            telemetry.addData("Dump Length2R", dumpLength2R);*/
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

        robot.turret.isNegative(false);
        //robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);//Left ? 79f : 79f);//(Left? 129f : 142f);//132
        drive.followTrajectorySequence(toSquare3);

        //robot.alignerInit();
        robot.claw.setAutoOpen();

        reset90(lp, Left, 5, false);
        //drive.followTrajectorySequence(toLine);

        long parkTime = 4500; //5500
        if (signalColors == SignalColors.Red)
            parkTime += Left ? 1000 : 2000;
        else if (signalColors == SignalColors.Blue)
            parkTime += 1000;
        else
            parkTime += Left ? 1000 : 1000;

        boolean firstTime = true;
        int numGrabbed = 0;
        int notGrab = 0;
        while(System.currentTimeMillis() - startTime < (30000-parkTime) && opModeIsActive() && cLevel >= 1 && notGrab < 2 && numGrabbed<5) {
            numGrabbed++;
            if (!grabbed)
                cLevel++;

            if (firstTime)
                firstTime = false;
            else
                reset90(lp, Left, cLevel, true);
            RobotLog.ii(TAG_SL, "cLevel: " + cLevel);
            robot.turret.turret.setTargetPositionPIDFCoefficients(1.9, 0, 0, 0);//1.4

            cLevel--;
            grabbed = false;

            float hSlidesOut = robot.hSlides.getPos();//153f;
            while (robot.sensorF.getDistance(DistanceUnit.CM) > 4 && hSlidesOut <= 141f) {//hSlidesOut >= hSlides.MIN+10) {
                hSlidesOut += 3;
                robot.hSlides.setPosition(hSlidesOut);
                lp.waitMillis(25);
            }

            RobotLog.ii(TAG_SL, "sensorF distance: " + robot.sensorF.getDistance(DistanceUnit.CM) + "hSlidesOut: " + hSlidesOut + "vSlides: currentPos R: " + robot.vSlides.slideRight.getCurrentPosition() +
                    "currentPos L: " + robot.vSlides.slideLeft.getCurrentPosition());
            robot.clawGrab();
            lp.waitMillis(350);
            robot.hSlides.setPosition((hSlidesOut - 6));
            robot.vSlides.moveTo(1030);//1430);//moveTo4();
            lp.waitMillis(200);
            robot.turret.moveTo((Left ? -167 : 167), turretPower);//, Left? false : true);//-180 : 180), turretPower); //-46
            //lp.waitMillis(100);

            if (robot.sensorF.getDistance(DistanceUnit.CM) < 10) {
                grabbed = true;
            } else
                notGrab++;

            robot.alignerOut();
            if(Left){robot.aligner.setRight();}
            robot.hSlides.setPosition(Left? dumpLength2L : dumpLength2R);//Left ? 137f : 127f);//(Left? 188f : 179f);//183f
            lp.waitMillis(750);
            RobotLog.ii(TAG_SL, "dump actual angle " + robot.turret.getCurrentAngle());

            if (robot.sensorF.getDistance(DistanceUnit.CM) < 10 && grabbed) {
                //robot.hSlides.setPosition(Left? 183f : 179f);
                drive.followTrajectorySequence(correct2);
                robot.vSlides.moveTo(850);
                robot.turret.setPower(0);
                lp.waitMillis(150);
            } else {
                lp.waitMillis(700);
            }
            //robot.alignerInit();
            robot.claw.setAutoOpen();
        }
        robot.alignerInit();
        robot.aligner.setLeft();

        robot.turret.turret.setTargetPositionPIDFCoefficients(7, 0, 0, 0);

        //lp.waitMillis(300);
        robot.hSlides.setPosition(hSlides.IN);
        lp.waitMillis(250);
        robot.clawGrab();
        robot.turret.moveTo(0, turretPower);//, Left? true : false);
        lp.waitMillis(500);

        lowerSlidesThread(lp, 1);
        if(signalColors == SignalColors.Red){
            if(Left) {
                drive.followTrajectorySequence(to3);
            } else { //special park
                drive.followTrajectorySequence(toR3);
            }
        } else if(signalColors == SignalColors.Green){
            if(Left) { //special park
                drive.followTrajectorySequence(to1);
            } else {
                drive.followTrajectorySequence(toR1);
            }
        } else{ //blue
            drive.followTrajectorySequence(to2);
        }
    }

    public void reset90(WaitLinear lp, boolean Left, int newL, boolean wait) throws InterruptedException {
        robot.turret.turret.setTargetPositionPIDFCoefficients(7,0,0,0);
        //if(!wait){
        //    robot.hSlides.setPosition(33f);//80f);
        //    lp.waitMillis(500);
        //} else {
        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);//140f);
        robot.aligner.setLeft();
        robot.alignerInit();
        lp.waitMillis(350);//600);
        //}

        robot.turret.moveTo(resetAngle, turretPower);//, Left? true : false);//(Left? -(resetAngle) : (resetAngle)), turretPower);

        lp.waitMillis(300);
        if(wait) {
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 105f:105f);//74f : 74f);
            robot.vSlides.moveTo(cone1+(interval*newL));
        } else{
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 113f:113f);
            //lp.waitMillis(200);//90f : 90f);
        }

        if(wait) {
            drive.followTrajectorySequence(correct);
            lp.waitMillis(300);
        } else {
            drive.followTrajectorySequence(toLine);
        }

        telemetry.addData("reset angle ", resetAngle);
        telemetry.update();
        while(!wait && robot.sensorF.getDistance(DistanceUnit.CM) > 8 && ((Left && robot.turret.getCurrentAngle() < 5) || (!Left && robot.turret.getCurrentAngle() > 5))){//robot.turret.getCurrentAngle()+360 < 370){
            robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setPower(Left? 0.12f : -0.12f);
            RobotLog.ii(TAG_SL, "current angle " + robot.turret.getCurrentAngle());
            telemetry.addData("current angle", robot.turret.getCurrentAngle());
            telemetry.addData("reset angle ", resetAngle);
            telemetry.addData("distance ", robot.sensorF.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.turret.setPower(0);
        if((Left && robot.turret.getCurrentAngle() >= 5) || (!Left && robot.turret.getCurrentAngle() <= 5)){//robot.turret.getCurrentAngle()+360 >= 370){
            resetAngle = 0;
            robot.turret.moveTo(resetAngle, turretPower);//, Left? false : true); //84.5, 81.4
        } else if(!wait) {
            resetAngle = robot.turret.getCurrentAngle() - (Left ? 0 : -5);
            robot.turret.moveTo(resetAngle, turretPower);//, Left? false : true);
        }
        RobotLog.ii(TAG_SL, "reset angle " + resetAngle);
        RobotLog.ii(TAG_SL, "reset actual angle " + robot.turret.getCurrentAngle());
    }

    public void lowerSlidesThread(WaitLinear lp, int level) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.85f,false, lp, this, robot, signalColors, cone1, interval, level);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}

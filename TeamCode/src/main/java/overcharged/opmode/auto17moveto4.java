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
import overcharged.drive.DriveConstants;
import overcharged.drive.SampleMecanumDrive;
import overcharged.linear.util.SelectLinear;
import overcharged.linear.util.WaitLinear;
//import overcharged.opmode.mSlidesThread;
import overcharged.test.EasyOpenCVExample;
import overcharged.test.SignalConePipeLine;
import overcharged.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="auto17moveto4")
public class auto17moveto4 extends LinearOpMode {

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
    int cone1 = -17;//22;//5;
    int interval = 55;//67
    boolean grabbed = true;
    double resetAngle = -6;
    int offset = 60;
    float dumpLengthL = 37f;
    float dumpLength2L = 119f;//172f;
    float dumpLengthR = 56f;
    float dumpLength2R = 135f;//172f;
    float close = 12f;
    float far = 15f;
    int Length = 0;
    int cLevel = 5;
    boolean Extra = true;
    double distance = 0.5;
    //int hSlidesReset = 150;

    TrajectorySequence toSquare3, toLine, toLine2, to1, to2, to3, toScore, correct, correct2, to2p2, score, toR1, toR3;
    Pose2d start = new Pose2d();

    Vector2d starting, line, p1, p2, p3, line2, p2p2, scorePos, pR3;
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
            //robot.clawGrab();
            robot.aligner.autoValue();

            Left = sl.selectPosition();
            if(!Left)
                resetAngle = 7;

            Length = sl.selectLength();
            if(Length == 2){
                dumpLengthL -= close;
                //dumpLength2L -= close;
                dumpLengthR -= close;
                //dumpLength2R -= close;
            } else if(Length == 1){
                dumpLengthL += far;
                //dumpLength2L += far;
                dumpLengthR += far;
                //dumpLength2R += far;
            }

            Extra = sl.selectExtraDistance();
            if(Extra)
                distance = 0.5;
            else
                distance = 0;

            double xVal = (Left? 50: 51);
            double yVal = (Left? 6: -6); //-6
            double yVal2 = (Left? -12: 10); //-6
            starting = new Vector2d(50, 1);
            line = new Vector2d(xVal, yVal);
            line2 = new Vector2d(xVal, yVal2);
            scorePos = new Vector2d(xVal, (Left? -15: 15));
            s3 = new Pose2d(xVal+distance,Left ? 1 : -1, Left? Math.toRadians(94) : Math.toRadians(-90));
            p1 = new Vector2d(Left? xVal-5 : xVal-1, (Left? 27 : 27));
            p2 = new Vector2d(xVal, (Left? 0 : 0)); //xVal-3
            p2p2 = new Vector2d(xVal-8, (Left? 0 : 3));
            p3 = new Vector2d(xVal-(Left? 0 : -1), -25);
            pR3 = new Vector2d(25, -25);

            toSquare3 = drive.trajectorySequenceBuilder(start)//65
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(65, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(s3)
                    .addSpatialMarker(new Vector2d((Left? 1 : 1),0), () -> {
                        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);
                    })
                    .addSpatialMarker(new Vector2d((Left? 10 : 15),0), () -> {
                        robot.vSlides.moveTo(1000);//1420);
                    })
                    .addSpatialMarker(new Vector2d((Left? 20 : 20), 0), () -> {
                        robot.turret.moveTo((Left? -175 : 172), turretPower);//, Left? false : true);//-154 : 151), turretPower);
                    })
                    .addSpatialMarker(new Vector2d((Left? 30 : 30),0), () -> {
                        robot.alignerOut();
                        if(Left){robot.aligner.setRight();}
                    })
                    .addSpatialMarker(new Vector2d((Left? 49.9+distance : 50.9+distance),0), () -> {
                        robot.vSlides.moveTo(850);
                        robot.turret.setPower(0);
                    })
                    /*.addSpatialMarker(new Vector2d((Left? 30 : 30),0), () -> {
                        robot.alignerOut();
                        if(Left){robot.aligner.setRight();}
                        robot.hSlides.setPosition(Left? dumpLengthL/3 : dumpLengthR/3);
                        robot.turret.turret.setTargetPositionPIDFCoefficients(1.5, 0, 0, 0);//1.4
                    })
                    .addSpatialMarker(new Vector2d((Left? 38 : 38),0), () -> {
                        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);
                    })
                    .addSpatialMarker(new Vector2d((Left? 49.9 : 50),0), () -> {
                        robot.vSlides.moveTo(1200);
                        robot.turret.setPower(0);
                    })*/
                    .build();

            toLine = drive.trajectorySequenceBuilder(toSquare3.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(line, Left? Math.toRadians(94) : Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(line.getX(),Left? 1: -1), () -> {
                        robot.vSlides.moveTo(cone1 + (interval * cLevel));
                    })
                    .build();/*
                    //.setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(starting, Left? Math.toRadians(88) : Math.toRadians(-88)))
                    //.strafeTo(line)
                    .build();*/

            correct = drive.trajectorySequenceBuilder(toSquare3.end())//new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(line, Left? Math.toRadians(94) : Math.toRadians(-90)))
                    .addSpatialMarker(new Vector2d(line2.getX(),Left? -6: 0), () -> {
                        robot.vSlides.moveTo(cone1 + (interval * cLevel));
                    })
                    .build();

            correct2 = drive.trajectorySequenceBuilder(new Pose2d(line.getX(), line.getY()+(Left? -1 : 1), Left? Math.toRadians(90) : Math.toRadians(-90)))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint((Left? 19 : 15), Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(line2, Left? Math.toRadians(94) : Math.toRadians(-90)))
                    /*.addSpatialMarker(new Vector2d(line2.getX(),Left? -9: 9), () -> {
                        robot.vSlides.moveTo(1460);//moveTo4();
                    })*/
                    .addSpatialMarker(new Vector2d(line2.getX(),Left? -2: -5.5), () -> {
                        robot.turret.turret.setTargetPositionPIDFCoefficients(1.5,0,0,0);
                        robot.alignerOut();
                        if(Left){robot.aligner.setRight();}
                        robot.hSlides.setPosition(Left? dumpLength2L : dumpLength2R);
                    })
                    .addSpatialMarker(new Vector2d(line2.getX(),Left? -6: 6), () -> {
                        robot.turret.turret.setTargetPositionPIDFCoefficients(0.7, 0, 0, 0);//1.4
                    })
                    .addSpatialMarker(new Vector2d(line2.getX(),Left? -11.5: 9.5), () -> {
                        robot.vSlides.moveTo(1180);
                    })
                    .build();

            score = drive.trajectorySequenceBuilder(toLine.end())
                    .lineTo(scorePos)
                    .build();

            to1 = drive.trajectorySequenceBuilder(correct2.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(p1, Math.toRadians(192)))
                    .addSpatialMarker(new Vector2d(p1.getX(), (Left? 4 : 17)), () -> {
                        lowerSlidesThread(lp, 1);
                    })
                    .addSpatialMarker(new Vector2d(p1.getX(), (Left? 24 : 24)), () -> {
                        if(robot.vSlides.switchSlideDown.isTouch())
                            robot.claw.setPosition(135f);
                    })
                    .build();

            toR1 = drive.trajectorySequenceBuilder(correct2.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(p1, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(p1.getX(),10.5), () -> {
                        robot.turret.moveTo(0, turretPower);
                    })
                    .addSpatialMarker(new Vector2d(p1.getX(), (Left? 21 : 21)), () -> {
                        lowerSlidesThread(lp, 1);})
                    .build();

            to2 = drive.trajectorySequenceBuilder(correct2.end())
                    .lineToLinearHeading(new Pose2d(p2, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(p2.getX(), Left? -3: p2.getY()+1), () -> {
                        lowerSlidesThread(lp, 1);
                    })
                    .build();

            to3 = drive.trajectorySequenceBuilder(correct2.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(p3, Math.toRadians(180)))
                    .addSpatialMarker(new Vector2d(p3.getX(),-15), () -> {
                        lowerSlidesThread(lp, 1);
                    })
                    .build();

            toR3 = drive.trajectorySequenceBuilder(correct2.end())
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(80, Math.PI * 2, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(new Pose2d(p3, Math.toRadians(180)))
                    /*.addSpatialMarker(new Vector2d(p3.getX(), 5), () -> {
                        robot.hSlides.setPosition(hSlides.IN);
                    })*/
                    .addSpatialMarker(new Vector2d(xVal-(Left? 2 : 2), -15), () -> {
                        lowerSlidesThread(lp, 1);
                    })
                    .addSpatialMarker(new Vector2d(p3.getX(), -24), () -> {
                        if(robot.vSlides.switchSlideDown.isTouch())
                            robot.claw.setPosition(135f);
                    })
                    .build();

            /*to2p2 = drive.trajectorySequenceBuilder(to2.end())
                    .lineTo(p2p2)
                    .build();*/

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

            drive.followerChange(0.1f);

            //detector.reset();
            telemetry.addData("Signal Color", signalColors);
            telemetry.addData("Left?", Left);
            //telemetry.addData("Aligner Value", robot.aligner.OUT);
            telemetry.addData("Extra", distance);
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
        //drive.followTrajectorySequence(toSquare3);

        robot.turret.isNegative(false);
        /*robot.hSlides.setPosition(Left? 40f : 40f);//dumpLengthL : dumpLengthR);//Left ? 79f : 79f);//(Left? 129f : 142f);//132
        robot.vSlides.moveTo(1460);
        robot.turret.moveTo((Left? -165 : 165), turretPower);*/
        drive.followTrajectorySequence(toSquare3);
        robot.vSlides.moveTo(850);

        //robot.alignerInit();
        /*robot.vSlides.moveTo(1200);
        robot.turret.setPower(0);
        lp.waitMillis(100);*/
        robot.claw.setAutoOpen();
        robot.aligner.setMiddle();
        //drive.followTrajectorySequence(toLine);

        reset90(lp, Left, 5, false);
        //drive.followTrajectorySequence(toLine);

        long parkTime = 4900; //5500
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
            robot.turret.turret.setTargetPositionPIDFCoefficients(1.5, 0, 0, 0);//1.4

            cLevel--;
            grabbed = false;

            float hSlidesOut = robot.hSlides.getPos();//153f;
            //lp.waitMillis(1000);
            while (robot.sensorF.getDistance(DistanceUnit.CM) > (Left ? 4 : 5) && hSlidesOut <= 158f) {//hSlidesOut >= hSlides.MIN+10) {
                hSlidesOut += 3;
                robot.hSlides.setPosition(hSlidesOut);
                lp.waitMillis(15);
            }

            RobotLog.ii(TAG_SL, "sensorF distance: " + robot.sensorF.getDistance(DistanceUnit.CM) + "hSlidesOut: " + hSlidesOut + "vSlides: currentPos R: " + robot.vSlides.slideRight.getCurrentPosition() +
                    "currentPos L: " + robot.vSlides.slideLeft.getCurrentPosition());
            robot.clawGrab();
            lp.waitMillis(500);
            robot.hSlides.setPosition((hSlidesOut - 6));
            robot.vSlides.moveTo(Left? 1455 : 1440);//moveTo4();
            lp.waitMillis(200);
            robot.hSlides.setPosition(hSlides.IN);

            robot.turret.moveTo((Left? -189 : 184), turretPower); //-46
            drive.followTrajectorySequence(correct2);
            //lp.waitMillis(150);

            if (robot.sensorF.getDistance(DistanceUnit.CM) < 4) {
                grabbed = true;
            } else {
                notGrab++;
            }
            RobotLog.ii(TAG_SL, "dump actual angle " + robot.turret.getCurrentAngle());

            if (robot.sensorF.getDistance(DistanceUnit.CM) < 10 && grabbed) {
                //robot.hSlides.setPosition(Left? 183f : 179f);
                robot.hSlides.setPosition(Left? (float)(dumpLength2L + 7) : (float)(dumpLength2R + 7));
                //robot.vSlides.moveTo(1200);
                lp.waitMillis(75);
                robot.turret.setPower(0);
            } else {
                robot.turret.setPower(0);
                lp.waitMillis(400);
            }
            //robot.alignerInit();
            robot.claw.setAutoOpen();
            robot.aligner.setMiddle();
        }
        //robot.aligner.setMiddle();
        robot.hSlides.setPosition(hSlides.IN);
        /*if(!(signalColors == SignalColors.Red && !Left)) {
            robot.hSlides.setPosition(hSlides.IN);
        } else {
            robot.hSlides.setPosition(Left? (float)(dumpLength2L-20) : (float)(dumpLength2R-20));
        }*/

        lp.waitMillis(100);

        if((signalColors == SignalColors.Red && !Left) || (signalColors == SignalColors.Blue)
                || (signalColors == SignalColors.Green && Left)) {
            robot.vSlides.moveTo3();
        } else {
            robot.vSlides.moveTo(650);
        }
        robot.alignerInit();
        robot.aligner.setLeft();

        robot.turret.turret.setTargetPositionPIDFCoefficients(9, 0, 0, 0);
        robot.clawGrab();
        if(!(signalColors == SignalColors.Green && !Left))
            robot.turret.moveTo(0, turretPower);

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
        //lp.waitMillis(300);
        /*robot.hSlides.setPosition(hSlides.IN);
        lp.waitMillis(550);
        robot.clawGrab();
        robot.turret.moveTo(0, turretPower);//, Left? true : false);*/

        /*robot.vSlides.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.vSlides.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.vSlides.slideMiddle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.vSlides.setPower(0);*/
        /*robot.vSlides.forcestop();
        robot.vSlides.reset(robot.vSlides.slideLeft);
        robot.vSlides.reset(robot.vSlides.slideMiddle);
        robot.vSlides.reset(robot.vSlides.slideRight);*/

        //lowerSlidesThread(lp, 1);
        robot.claw.setPosition(135f);
        lp.waitMillis(30000-System.currentTimeMillis()+startTime);
    }

    public void reset90(WaitLinear lp, boolean Left, int newL, boolean wait) throws InterruptedException {
        /*robot.turret.turret.setTargetPositionPIDFCoefficients(7,0,0,0);

        robot.hSlides.setPosition(Left? dumpLengthL : dumpLengthR);//140f);
        if(wait)
            robot.aligner.setLeft();

        robot.alignerInit();
        lp.waitMillis(350);

        robot.turret.moveTo(resetAngle, turretPower);//, Left? true : false);//(Left? -(resetAngle) : (resetAngle)), turretPower);

        lp.waitMillis(300);
        if(wait) {
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 120f:120f);//74f : 74f);
            robot.vSlides.moveTo(cone1+(interval*newL));
        } else{
            robot.aligner.setLeft();
            robot.turret.turret.setTargetPositionPIDFCoefficients(3,0,0,0);
            robot.hSlides.setPosition(Left? 120f:120f);//113
            //lp.waitMillis(200);//90f : 90f);
        }

        if(wait) {
            drive.followTrajectorySequence(correct);
            lp.waitMillis(300);
        } else {
            drive.followTrajectorySequence(toLine);
        }*/

        robot.turret.turret.setTargetPositionPIDFCoefficients(7,0,0,0);
        robot.alignerInit();
        lp.waitMillis(350);//600);
        robot.hSlides.setPosition(hSlides.IN);//140f);
        robot.aligner.setLeft();

        robot.turret.moveTo(resetAngle, turretPower);//, Left? true : false);//(Left? -(resetAngle) : (resetAngle)), turretPower);

        if(wait) {
            lp.waitMillis(400);
            robot.turret.turret.setTargetPositionPIDFCoefficients(2.5, 0, 0, 0);
            robot.hSlides.setPosition(Left? 125f : 125f);//136f//(Left? 143f : 145f);//(Left ? 85f : 100f));
            drive.followTrajectorySequence(correct);
            //lp.waitMillis(300);
            //robot.vSlides.moveTo(cone1 + (interval * newL));
            //lp.waitMillis(100);
        } else {
            lp.waitMillis(550);
            robot.turret.turret.setTargetPositionPIDFCoefficients(2, 0, 0, 0);
            robot.hSlides.setPosition(Left? 142f : 137f);//136f//(Left? 143f : 145f);//(Left ? 85f : 100f));
            drive.followTrajectorySequence(toLine);//toLine);
        }

        telemetry.addData("reset angle ", resetAngle);
        telemetry.update();
        while(!wait && robot.sensorF.getDistance(DistanceUnit.CM) > 10 && ((Left && robot.turret.getCurrentAngle() < 5) || (!Left && robot.turret.getCurrentAngle() > -5))){//robot.turret.getCurrentAngle()+360 < 370){
            robot.turret.turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turret.setPower(Left? 0.09f : -0.09f);
            RobotLog.ii(TAG_SL, "current angle " + robot.turret.getCurrentAngle());
            telemetry.addData("current angle", robot.turret.getCurrentAngle());
            telemetry.addData("reset angle ", resetAngle);
            telemetry.addData("distance ", robot.sensorF.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        if(!wait){
            RobotLog.ii(TAG_SL, "reset angle detected distance " + robot.sensorF.getDistance(DistanceUnit.CM));
        }
        robot.turret.setPower(0);
        if((Left && robot.turret.getCurrentAngle() >= 5) || (!Left && robot.turret.getCurrentAngle() <= -5)){//robot.turret.getCurrentAngle()+360 >= 370){
            resetAngle = 0;
            robot.turret.moveTo(resetAngle, turretPower);//, Left? false : true); //84.5, 81.4
        } else if(!wait) {
            resetAngle = robot.turret.getCurrentAngle() - (Left ? 1 : 0);
            robot.turret.moveTo(resetAngle, turretPower);//, Left? false : true);
        }
        RobotLog.ii(TAG_SL, "reset angle " + resetAngle);
        RobotLog.ii(TAG_SL, "reset actual angle " + robot.turret.getCurrentAngle());
    }

    public void lowerSlidesThread(WaitLinear lp, int level) { // asynchronously start raising the slides
        Runnable lowerSlidesThread = new vSlidesThread(0.6f, false, lp, this, robot, signalColors, cone1, interval, level);
        Thread thread = new Thread(lowerSlidesThread);
        thread.start();
    }
}

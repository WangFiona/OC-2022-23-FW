package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

import overcharged.components.Button;
import overcharged.components.DuckDetector;
import overcharged.components.DuckPositions;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
import overcharged.linear.util.SelectLinear;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Duck Detection", group = "Test")
public class DuckDetection extends OpMode {

    private static String csvSeperator = ",";
    private ElapsedTime runtime = new ElapsedTime();
    DuckDetector detector;
    OpenCvWebcam webcam;
    DuckPositions duckPositions = DuckPositions.A;
    boolean initDetector = false;
    boolean isRed = false;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    double offset=0;
    double C1 = 0.01;
    double C2 = 0.22;
    double C3 = 0.45;
    double RS = 0.48;

    /**
     * Camera Initialization Function
     */
    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDevice();
    }

    @Override
    public void init() {
        initCamera();
        initDetector = false;
        this.detector = new DuckDetector();
        this.detector.useDefaults();
        setRange();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webcam.showFpsMeterOnViewport(false);
        long time1 = System.currentTimeMillis();
        duckPositions = detector.getDuckPositions();

        telemetry.addData("Initial ring Position=", duckPositions);
        telemetry.update();
    }

    public void stop() {
        //without the following 2 lines the next run will show blue screen
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        super.stop();
    }

    //blue warehouse
    //private void setRange() {this.detector.setRange(0.07, 0.28, 0.51, 0.53);}

    //blue storage
    //private void setRange() {this.detector.setRange(0.34, 0.6, 0.84, 0.55);}

    //red warehouse
    //private void setRange() {this.detector.setRange(0.34, 0.58, 0.84, 0.54);}

    //red storage
    //private void setRange() {this.detector.setRange(0.03, 0.26, 0.51, 0.54);}

    //private void setRange(){this.detector.setRange(0.33, 0.57, 0.83, 0.45); } //red
    //private void setRange(){this.detector.setRange(0.01, 0.22, 0.45, 0.48); } //blue
    private void setRange(){this.detector.setRange(C1, C2, C3, RS); } //blue

    @Override
    public void loop() {
        telemetry.addData("Position", (isRed ? "Red" : "Blue"));
        telemetry.addData("Toggle Position", "Right Bumper");
        telemetry.addData("Stop", "Back");
        telemetry.addData("Offset", "%.2f", offset);
        long timeStamp = System.currentTimeMillis();
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            telemetry.update();
            stop();
        }

        if (isRed && gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timeStamp)) {
            isRed = false;
            initDetector = true;
            this.detector.setRange(0.07+offset, 0.28+offset, 0.51+offset, 0.43); //blue
        }
        else if(!isRed && gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timeStamp)) {
            isRed = true;
            initDetector = true;
            this.detector.setRange(C1+offset, C2+offset, C3+offset, RS);//red
        }

        if(gamepad1.b && Button.BTN_PLUS.canPressShort(timeStamp)){
            if((isRed && offset<0.17) || (!isRed && offset<0.49)) {
                offset += 0.02;
                telemetry.addData("Offset", "%.2f", offset);
            }
        }
        else if(gamepad1.x && Button.BTN_MINUS.canPressShort(timeStamp)){
            if((isRed && offset> -0.33) || (!isRed && offset> -0.07)) {
                offset -= 0.02;
                telemetry.addData("Offset", "%.2f", offset);
            }
        }

        if(isRed){
            this.detector.setRange(0.33+offset, 0.57+offset, 0.83+offset, 0.45);//red
        } else{
            this.detector.setRange(C1+offset, C2+offset, C3+offset, RS); //blue
        }

        if (initDetector) {
            initDetector = false;
            webcam.stopStreaming();
            this.detector = new DuckDetector();
            this.detector.useDefaults();
            setRange();
            webcam.setPipeline(detector);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        telemetry.addData("Duck Position", detector.getDuckPositions());
        telemetry.addData("Position X", detector.getScreenPosition().x + " Y:" + detector.getScreenPosition().y);
        telemetry.addData("Rect Width", detector.foundRectangle().width + " Height:", detector.foundRectangle().height);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.update();

        if (gamepad1.a) {
            webcam.stopStreaming();
        } else if (gamepad1.x) {
            webcam.pauseViewport();
            detector.reset();
        } else if (gamepad1.y) {
            webcam.resumeViewport();
        }
    }
}
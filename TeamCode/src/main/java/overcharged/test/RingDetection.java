package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

import overcharged.components.Button;
import overcharged.components.RingDetector;
import overcharged.components.RingPosition;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Ring Detection", group = "Test")
//@Disabled
public class RingDetection extends OpMode {

    private static String csvSeperator = ",";
    private ElapsedTime runtime = new ElapsedTime();
    RingDetector detector;
    //OpenCvInternalCamera webcam;
    //OpenCvCamera webcam;
    OpenCvWebcam webcam;
    RingPosition ringPosition = RingPosition.A;
    boolean initDetector = false;
    boolean isRed = true;
    EasyOpenCVExample.RingDeterminationPipeline pipeline;
    //boolean isNear = false;

//    Persist persist;
//    private double r_n_start = 0.27;
//    private double r_n_first = 115;
//    private double r_n_second = 28;
//    private double b_n_start = 0.3;
//    private double b_n_first = 33;
//    private double b_n_second = 120;

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
        //persist = new Persist(hardwareMap.appContext, "OverchargedSkystone.txt");
        initDetector = false;
        // numbers for far position
        //this.detector.setRedRange(0.13, 25, 112);
        //this.detector.setBlueRange(0.4, 122, 27);

        this.detector = new RingDetector(isRed);
        this.detector.useDefaults();
        setRange();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        webcam.showFpsMeterOnViewport(false);
        long time1 = System.currentTimeMillis();
        //long currentTime = time1;
        //while (currentTime - time1 < SkystoneDetector.DETECTION_WAIT_TIME) {
        ringPosition = detector.getRingPosition();
        //    currentTime = System.currentTimeMillis();
        //}
        telemetry.addData("Initial ring Position=", ringPosition);
        telemetry.update();
    }

    public void stop() {
        //without the following 2 lines the next run will show blue screen
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        super.stop();
    }

    private void setRange() {
        // numbers for far position
        this.detector.setRedRange(0.18, 115, 28); //0.52
        this.detector.setBlueRange(0.15, 30, 118);
        //this.detector.setRedRange(0.26, 115, 28);
        //this.detector.setBlueRange(0.23, 33, 120);
        //competition this.detector.setRedRange(0.33, 115, 28);
        //            this.detector.setBlueRange(0.23, 33, 120);
        //numbers for near position:
        //this.detector.setRedRange(0.13, 25, 112);
        //this.detector.setBlueRange(0.4, 122, 27);
        //this.detector.setRedRange(0.27, 115, 28);
        //this.detector.setBlueRange(0.3, 33, 120);
    }
//    @Override
//    public void init() {
//    }

//    @Override
//    public void loop()
//    {
//        long timeStamp = System.currentTimeMillis();
//        if (gamepad1.b && Button.BTN_BACK.canPress(timeStamp)) {
//            List<Double> list = new ArrayList<Double>();
//            list.add(r_n_start);
//            list.add(r_n_first);
//            list.add(r_n_second);
//            list.add(b_n_start);
//            list.add(b_n_first);
//            list.add(b_n_second);
//            write(list);
//            read();
//        }
//    }


    @Override
    public void loop() {
        telemetry.addData("Position", (isRed ? "Right" : "Left"));
        telemetry.addData("Toggle Position", "Right Bumper");
        telemetry.addData("Stop", "Back");
        long timeStamp = System.currentTimeMillis();
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            telemetry.update();
            stop();
        }
        if (gamepad1.right_bumper && Button.BTN_SLOW_MODE.canPress(timeStamp)) {
            isRed = !isRed;
            initDetector = true;
            setRange();
        } /*else if (gamepad1.left_bumper && Button.BTN_SUPERSLOW_MODE.canPress(timeStamp)) {
            isNear = !isNear;
            initDetector = true;
        }*/
//        if (gamepad1.dpad_left) {
//            if (isRed) {
//                r_n_start = r_n_start - 1/100;
//                r_n_first--;
//                r_n_second--;
//            } else {
//                b_n_start = b_n_start - 1/100;
//                b_n_first--;
//                b_n_second--;
//            }
//        } else if (gamepad1.dpad_right) {
//            if (isRed) {
//                r_n_start = r_n_start + 1/100;
//                r_n_first++;
//                r_n_second++;
//            } else {
//                b_n_start = b_n_start - 1/100;
//                b_n_first++;
//                b_n_second++;
//            }
//        }

        if (initDetector) {
            initDetector = false;
            webcam.stopStreaming();
            this.detector = new RingDetector(isRed);
            this.detector.useDefaults();
            setRange();
            //this.detector.setRedRange(r_n_start, r_n_first, r_n_second);
            //this.detector.setBlueRange(b_n_start, b_n_first, b_n_second);
            webcam.setPipeline(detector);
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }

        telemetry.addData("Ring Position", detector.getRingPosition());
        telemetry.addData("Position X", detector.getScreenPosition().x + " Y:" + detector.getScreenPosition().y);
        telemetry.addData("Rect Width", detector.foundRectangle().width + " Height:", detector.foundRectangle().height);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));

//        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
//        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
//        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
//        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//        telemetry.addData("Classification", detector.getSkystoneState().toString());
        telemetry.update();

//        if (gamepad1.b) {
//            List<Double> list = new ArrayList<Double>();
//            list.add(r_n_start);
//            list.add(r_n_first);
//            list.add(r_n_second);
//            list.add(b_n_start);
//            list.add(b_n_first);
//            list.add(b_n_second);
//            write(list);
//        }
        if (gamepad1.a) {
            webcam.stopStreaming();
        } else if (gamepad1.x) {
            webcam.pauseViewport();
            detector.reset();
        } else if (gamepad1.y) {
            webcam.resumeViewport();
        }
    }

//    private boolean write(List<Double> list)  {
//        try {
//            StringBuilder sb = new StringBuilder();
//            for (Double i : list) {
//                if (i > 0)
//                    sb.append(csvSeperator);
//                sb.append(i);
//            }
//            persist.write(sb.toString());
//            telemetry.addData("Success", "Done writing to storage");
//        } catch (Exception e) {
//            telemetry.addData("Error", "writeToFile Failed:" + e.getMessage());
//            return false;
//        }
//        telemetry.update();
//        return true;
//    }
//
//    private Double StringToDouble(String val) {
//        double myNum = 0;
//        try {
//            myNum = Double.parseDouble(val);
//        } catch(NumberFormatException nfe) {
//            telemetry.addData("Erro", "Could not parse " + nfe);
//        }
//        return myNum;
//    }
//
//    private List<Double> read()
//    {
//        List<Double> resultList = new ArrayList<Double>();
//        try {
//            String csvLine = persist.read(false);
//            String[] row = csvLine.split(csvSeperator);
//            for(String s : row) {
//                resultList.add(StringToDouble(s));
//            }
//            telemetry.addData("READ", csvLine);
//            telemetry.update();
//        } catch (Exception e) {
//            // Error occurred when opening raw file for reading.
//            telemetry.addData("Error", "in reading CSV file: " + e);
//        }
//        telemetry.update();
//        return resultList;
//    }
}
package overcharged.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Computer Vision")
public class CVTest extends LinearOpMode {
    SignalConePipeLine pipeline;
    int displayID;

    public void runOpMode() {
        //easyCV.runOpMode();
        displayID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        final OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), displayID);

        pipeline = new SignalConePipeLine();
        cam.setPipeline(pipeline);
        pipeline.isLeft(false);
        //cam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() //calls when camera opens
            {
                cam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) //calls if camera did not open
            {
                System.out.println(errorCode);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Color", pipeline.getColor());
            telemetry.update();

            sleep(50);
        }
    }
} // end of CVTest

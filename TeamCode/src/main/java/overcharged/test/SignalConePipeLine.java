package overcharged.test;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import overcharged.components.DuckPositions;
import overcharged.components.SignalColors;

public class SignalConePipeLine extends OpenCvPipeline {
    int avgR, avgG, avgB;
    String color;
    private SignalColors signalColors = SignalColors.Red;
    private Rect foundRect = new Rect(); // Found rect
    private Point screenPosition = new Point(); // Center screen position of the block

    /*HOW TO DO IT:
    figure out where the cone is gonna sit
    then use java math.max (or similar) to figure out if the cone has the red, green or blue
     side facing us*/
    @Override
    public Mat processFrame(Mat input){
        //coordinates of top-left and bottom right points in rectangle
        double x1 = 0.56;
        double x2 = 0.64;
        double y1 = 0.6;
        double y2 = 0.43;
        Point tl  = new Point(x1 * input.cols(),y1 * input.rows());
        Point br = new Point(x2 * input.cols(), y2 * input.rows());
        Mat rawImage = input.submat((int)(y2 * input.rows()), (int)(y1 * input.rows()), (int)(x1 * input.cols()), (int)(x2 * input.cols()));

        avgR = (int) Core.mean(rawImage).val[0];
        avgG = (int) Core.mean(rawImage).val[1];
        avgB = (int) Core.mean(rawImage).val[2];
        int maxRG = Math.max(avgR, avgG);
        int max = Math.max(maxRG, avgB);

        if (max == avgR) {
            color = "RED";
            signalColors = SignalColors.Red;
        } else if (max == avgG) {
            color = "GREEN";
            signalColors = SignalColors.Green;
        } else if (max == avgB) {
            color = "BLUE";
            signalColors = SignalColors.Blue;
        }
        //telemetry.update();
        Imgproc.rectangle(input, tl, br, new Scalar(0,0,255),3);
        return input;
    }

    public String getColor() {
        return color;
    }
    public SignalColors getSignalColors() {
        return signalColors;
    }

    public void reset() {
        //found = false;
        foundRect = new Rect();
        screenPosition = new Point();
        signalColors = SignalColors.Red;
    }
}
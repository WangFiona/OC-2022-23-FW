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
    boolean Left = true;
    private SignalColors signalColors = SignalColors.Red;
    private Rect foundRect = new Rect(); // Found rect
    private Point screenPosition = new Point(); // Center screen position of the block
    double x1 = 0.54;
    double x2 = 0.62;
    double y1 = 0.6;
    double y2 = 0.43;

    /*HOW TO DO IT:
    figure out where the cone is gonna sit
    then use java math.max (or similar) to figure out if the cone has the red, green or blue
     side facing us*/

    public void isLeft(boolean L){
        Left = L;
    }

    @Override
    public Mat processFrame(Mat input){
        //coordinates of top-left and bottom right points in rectangle
        if(Left){
            x1 = 0.484;//0.48;
            x2 = 0.564;//0.56;
            y1 = 0.596;//0.48;
            y2 = 0.426;//0.31;
             /*x1 = 0.56;
             x2 = 0.64;
             y1 = 0.6;
             y2 = 0.43;*/
        } else {
            x1 = 0.62;
            x2 = 0.7;
            y1 = 0.508;
            y2 = 0.38;
             /*x1 = 0.59;
             x2 = 0.67;
             y1 = 0.6;
             y2 = 0.43;*/
        }
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
package overcharged.components;

import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static overcharged.config.RobotConstants.TAG_TD;

/**
 * NOTE:
 * OpenCV is an image processing library. It contains a large collection of image processing functions.
 * To solve a computational challenge, most of the time you will end up using multiple functions of the library.
 * Because of this, passing images to functions is a common practice.
 * We should not forget that we are talking about image processing algorithms, which tend to be quite computationally heavy.
 * The last thing we want to do is further decrease the speed of your program by making unnecessary copies of potentially large images.
 */

//Red=left Blue=Right
public class DuckDetector extends DogeCVDetector {
    public static double C_ONE = 0.2;
    public static double C_TWO = 0.45;
    public static double C_THREE = 0.7;
    public static double C_DIFF = 0.06;

    public static double R_TOP = 0.7;
    public static double R_DIFF = 0.09;
    public static double R_BOTTOM = R_TOP+R_DIFF;

    public static long DETECTION_WAIT_TIME = 800;

    // Results of the detector
    private Point screenPosition = new Point(); // Center screen position of the block
    private Rect foundRect = new Rect(); // Found rect
    private DuckPositions duckPositions = DuckPositions.A;

    private Mat rawImage1 = new Mat();
    private Mat rawImage2 = new Mat();
    private Mat rawImage3 = new Mat();
    private Mat outMat = new Mat();


    public Point getScreenPosition() {
        return screenPosition;
    }
    public Rect foundRectangle() {
        return foundRect;
    }
    public DuckPositions getDuckPositions() {
        return duckPositions;
    }


    public DuckDetector() {
        detectorName = "12599 Duck Detector";
    }

    public void reset() {
        found = false;
        foundRect = new Rect();
        screenPosition = new Point();
        duckPositions = DuckPositions.A;
    }

    @Override
    public Mat process(Mat input) {
        // These need to be computed at program start to prevent FTC dashboard from changing the values
        // of START_ROW and the like between when we crop RawImage and when we copy it and output it
        RobotLog.ii(TAG_TD, "colorGray=" + input.rows());
        int SC1 = (int)(input.cols() * C_ONE);
        int SC2 = (int)(input.cols() * C_TWO);
        int SC3 = (int)(input.cols() * C_THREE);

        int EC1 = (int)(input.cols() * (C_ONE+C_DIFF));
        int EC2 = (int)(input.cols() * (C_TWO+C_DIFF));
        int EC3 = (int)(input.cols() * (C_THREE+C_DIFF));

        int SR = (int)(input.rows() * R_TOP);
        int ER = (int)(input.rows() * R_BOTTOM);

        Scalar A, B, C;
        double colorA, colorB, colorC, AB, AC, BC, tolerance=30, tolerance2=35;

        rawImage1 = input.submat(SR, ER, SC1, EC1);
        rawImage2 = input.submat(SR, ER, SC2, EC2);
        rawImage3 = input.submat(SR, ER, SC3, EC3);

        A = Core.mean(rawImage1);
        B = Core.mean(rawImage2);
        C = Core.mean(rawImage3);

        /*colorA = (A.val)[0];
        colorB = (B.val)[0];
        colorC = (C.val)[0];
        AB = Math.abs(colorA-colorB);
        AC = Math.abs(colorA-colorC);
        BC = Math.abs(colorB-colorC);

        if(BC<AB && BC<AC) {
            duckPositions = DuckPositions.A;
        } else if(AC<AB && AC<BC) {
            duckPositions = DuckPositions.B;
        } else {
            duckPositions = DuckPositions.C;
        }*/

        colorA = Math.abs((A.val)[1]-(A.val)[0]);
        colorB = Math.abs((B.val)[1]-(B.val)[0]);
        colorC = Math.abs((C.val)[1]-(C.val)[0]);

        if(colorA > colorB && colorA > colorC){
            duckPositions = DuckPositions.A;
        } else if(colorB > colorA && colorB > colorC){
            duckPositions = DuckPositions.B;
        } else {
            duckPositions = DuckPositions.C;
        }

        RobotLog.ii(TAG_TD, "Colors Log color A: " + (A.val)[0] + " : " + (A.val)[1] + " : " + (A.val)[2] + " total= " + colorA);
        RobotLog.ii(TAG_TD, "Colors Log color B: " + (B.val)[0] + " : " + (B.val)[1] + " : " + (B.val)[2] + " total= " +  colorB);
        RobotLog.ii(TAG_TD, "Colors Log color C: " + (C.val)[0] + " : " + (C.val)[1] + " : " + (C.val)[2] + " total= " + colorC);

        RobotLog.ii(TAG_TD, "colorA: " + (255 * (1-(A.val)[0]) * (1-(A.val)[3])) + " : " + (255*(1-(A.val)[1])*(1-(A.val)[3])) + " : " + (255*(1-(A.val)[2])*(1-(A.val)[3])));
        RobotLog.ii(TAG_TD, "colorB: " + (255*(1-(B.val)[0])*(1-(B.val)[3])) + " : " + (255*(1-(B.val)[1])*(1-(B.val)[3])) + " : " + (255*(1-(B.val)[2])*(1-(B.val)[3])));
        RobotLog.ii(TAG_TD, "colorC: " + (255*(1-(C.val)[0])*(1-(C.val)[3])) + " : " + (255*(1-(C.val)[1])*(1-(C.val)[3])) + " : " + (255*(1-(C.val)[2])*(1-(C.val)[3])));



        input.copyTo(outMat);
        Point t1 = new Point(SC1, SR);
        Point b1 = new Point(EC1, ER);
        Rect rect1 = new Rect(t1, b1);

        Point t2 = new Point(SC2, SR);
        Point b2 = new Point(EC2, ER);
        Rect rect2 = new Rect(t2, b2);

        Point t3 = new Point(SC3, SR);
        Point b3 = new Point(EC3, ER);
        Rect rect3 = new Rect(t3, b3);

        Imgproc.rectangle(outMat, rect1.tl(), rect1.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, rect2.tl(), rect2.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, rect3.tl(), rect3.br(), new Scalar(0,0,255),2);
        return outMat;
    }

    public void setRange(double C1, double C2, double C3, double RS){
        C_ONE = C1;
        C_TWO = C2;
        C_THREE = C3;
        R_TOP = RS;
        R_BOTTOM = R_TOP+R_DIFF;
    }

    @Override
    public void useDefaults() {
    }
}

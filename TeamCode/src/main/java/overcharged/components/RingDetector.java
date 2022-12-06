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
public class RingDetector extends DogeCVDetector {
    public static double START_ROW = 0.3; //0.455
    public static double END_ROW = 0.9; //0.63
    public static double BLUE_START_ROW = 0.38; //0.455
    public static double BLUE_END_ROW = 0.98; //0.63
    public static double COL_diff = 0.2; //0.15,0.18
    public double START_COL_RED = 0.1;
    public double END_COL_RED = START_COL_RED + COL_diff;
    public double START_COL_BLUE = 0.18;
    public double END_COL_BLUE = START_COL_BLUE + COL_diff;

    /*public double START_FAR_COL_RED = 0;
    public double END_FAR_COL_RED = START_FAR_COL_RED + COL_diff;
    public double START_FAR_COL_BLUE = 0.5;
    public double END_FAR_COL_BLUE = START_FAR_COL_BLUE + COL_diff;*/
    ///This is the minimum time needed to wait for the detection to work correctly
    public static long DETECTION_WAIT_TIME = 800;

    //Put the values found during testing here
    public double BLUE_FIRST_CUTOFF = 125;
    public double BLUE_SECOND_CUTOFF = 35;

    public double RED_FIRST_CUTOFF = 28;
    public double RED_SECOND_CUTOFF = 113;

    // Results of the detector
    private Point screenPosition = new Point(); // Center screen position of the block
    private Rect foundRect = new Rect(); // Found rect
    private RingPosition ringPosition = RingPosition.A;
    private boolean isRed = true;

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
    public RingPosition getRingPosition() {
        return ringPosition;
    }


    public RingDetector(boolean isRed) {
        detectorName = "12599 Ring Detector";
        this.isRed = isRed;
        //this.isNear = isNear;
    }

    public void reset() {
        found = false;
        foundRect = new Rect();
        screenPosition = new Point();
        ringPosition = RingPosition.A;
    }

    @Override
    public Mat process(Mat input) {
        // These need to be computed at program start to prevent FTC dashboard from changing the values
        // of START_ROW and the like between when we crop RawImage and when we copy it and output it
        RobotLog.ii(TAG_TD, "colorGray=" + input.rows());
        int startRow = (int) (input.rows() * (this.isRed ? START_ROW : BLUE_START_ROW));
        int endRow = (int) (input.rows() * (this.isRed ? END_ROW : BLUE_END_ROW));
        int startCol = (int) (input.cols() * (this.isRed ? START_COL_RED : START_COL_BLUE));
        int endCol = (int) (input.cols() * (this.isRed ? END_COL_RED : END_COL_BLUE));
        int margin = 30;
        //int halfCol = (startCol+endCol)/2;
        int halfRow = (startRow+endRow)/2 - 10;
        int lowRow = halfRow+margin;
        double greyRed = 0.7;
        double greyBlue = 0.7;
        int greyCol = (int) (input.cols() * (this.isRed ? greyRed : greyBlue));
        double greyColLength = 0.1;
        int greyEndCol = (int)(input.cols() * (greyColLength + (this.isRed ? greyRed : greyBlue)));
        Scalar meanBottom, meanTop, meanGray;
        double colorBottom, colorTop, colorGray, tolerance=30, tolerance2=35;

        /*rawImage1 = input.submat(startRow, endRow, startCol, halfCol-margin);
        rawImage2 = input.submat(startRow, endRow, halfCol+margin, endCol);*/

        rawImage1 = input.submat(startRow, halfRow, startCol, endCol);
        rawImage2 = input.submat(lowRow, endRow, startCol, endCol);
        rawImage3 = input.submat(halfRow, endRow, greyCol, greyEndCol);


        Point tl = new Point(startCol, startRow);
        //Point br = new Point(halfCol-margin, endRow);
        Point br = new Point(endCol, halfRow);
        Rect rect1 = new Rect(tl,br);

        /*tl.y = startRow;
        tl.x = halfCol+margin;*/
        tl.y = lowRow;
        tl.x = startCol;
        br.y = endRow;
        br.x = endCol;
        Rect rect2 = new Rect(tl,br);

        meanBottom = Core.mean(rawImage2);
        meanTop = Core.mean(rawImage1);
        meanGray = Core.mean(rawImage3);
        colorBottom = (meanBottom.val)[2];//+(mean1.val)[1]; //+(mean1.val)[2];
        colorTop = (meanTop.val)[2];//+(mean2.val)[1]; //+(mean2.val)[2];
        colorGray = (meanGray.val)[2];//+(meanGray.val)[1]; //+(mean2.val)[2];

        RobotLog.ii(TAG_TD, "Colors Log colorbottom: " + (meanBottom.val)[0] + " : " + (meanBottom.val)[1] + " : " + (meanBottom.val)[2] + " total= " + colorBottom);
        RobotLog.ii(TAG_TD, "Colors Log colortop: " + (meanTop.val)[0] + " : " + (meanTop.val)[1] + " : " + (meanTop.val)[2] + " total= " + colorTop);
        RobotLog.ii(TAG_TD, "Colors Log colorGray: " + (meanGray.val)[0] + " : " + (meanGray.val)[1] + " : " + (meanGray.val)[2] + " total= " + colorGray);

        found = true;
        if (colorTop - colorBottom > tolerance*2) { //1 ring
            ringPosition = RingPosition.B;
            foundRect = rect2;
        } else if (colorGray - colorTop > tolerance2) { //4 rings
            ringPosition = RingPosition.C;
            foundRect = rect1;
        } else { //no ring
            ringPosition = RingPosition.A; //0 rings
            foundRect.height = 0;
            foundRect.width = 0;
            found = false;
        }
        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);

        input.copyTo(outMat);
        Point trueTL = new Point(startCol, startRow);
        Point trueBR1 = new Point(endCol, halfRow);
        Point trueBR = new Point(endCol, endRow);
        Point trueTL2 = new Point(startCol, lowRow);
        Rect crop = new Rect(trueTL, trueBR1);
        Rect crop2 = new Rect(trueTL2, trueBR);
        Point grey = new Point(greyCol, halfRow);
        Point grey2 = new Point(greyEndCol, endRow);
        Rect greyRect = new Rect(grey, grey2);

        Imgproc.rectangle(outMat, crop.tl(), crop.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, crop2.tl(), crop2.br(), new Scalar(0,0,255),2);
        Imgproc.rectangle(outMat, greyRect.tl(), greyRect.br(), new Scalar(0,0,255),2);
        if (found) {
            Imgproc.rectangle(outMat, foundRect, new Scalar(255, 0, 0), 4);
            screenPosition.x = foundRect.x + foundRect.width / 2;
            screenPosition.y = foundRect.y + foundRect.height / 2;
        } else {
            screenPosition.x = 0;
            screenPosition.y = 0;
        }

        return outMat;
    }

    public void setRedRange(double redStart, double redFirst, double redSecond){
        START_COL_RED = redStart;
        END_COL_RED = START_COL_RED + COL_diff;
        RED_FIRST_CUTOFF = redFirst;
        RED_SECOND_CUTOFF = redSecond;
    }

    public void setBlueRange(double blueStart, double blueFirst, double blueSecond){
        START_COL_BLUE = blueStart;
        END_COL_BLUE = START_COL_BLUE + COL_diff;
        BLUE_FIRST_CUTOFF = blueFirst;
        BLUE_SECOND_CUTOFF = blueSecond;
    }

    @Override
    public void useDefaults() {
    }
}

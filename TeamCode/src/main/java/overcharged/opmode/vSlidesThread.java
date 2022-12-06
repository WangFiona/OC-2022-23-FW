package overcharged.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import overcharged.components.DuckPositions;
import overcharged.components.RobotMecanum;
import overcharged.components.SignalColors;
import overcharged.linear.components.Robot6WheelLinear;
import overcharged.linear.components.RobotTankMecanumLinear;
import overcharged.linear.util.WaitLinear;

import static overcharged.config.RobotConstants.TAG_A;

public class vSlidesThread implements Runnable {
    private RobotMecanum robot;
    private WaitLinear lp;
    private LinearOpMode opMode;

    double startencoder;
    double c1;
    int level;
    double interval;
    //double level3;
    boolean raiseUp=true;
    //boolean switchBroken = false;
    long slideDownTime;
    int cycleNumber=0;

    private SignalColors signalColors = SignalColors.Red;

    public vSlidesThread(boolean up, WaitLinear wl, LinearOpMode mode, RobotMecanum r, SignalColors d, double cone1, double i, int l) {
        // store parameter for later use
        c1 = cone1;
        interval = i;
        level = l;
        raiseUp=up;
        lp = wl;
        opMode = mode;
        robot = r;
        signalColors=d;

        /*startencoder=se;
        cycleNumber=cN;*/
    }

    public void run() {
        try {
            if (!opMode.isStopRequested()) {
                RobotLog.v(TAG_A, "Start raising slides");
                long startTime = System.currentTimeMillis();

                if(raiseUp==true){
                    slideUp(lp);
                } else{
                    slideDown(lp);
                }

                long TimeElapsed = System.currentTimeMillis() - startTime;
                RobotLog.ii(TAG_A, "Stop raising slides " + (System.currentTimeMillis() - startTime) + " milliseconds");
            }
        } catch (InterruptedException e) {
            RobotLog.ii(TAG_A, "Error: " + e.getStackTrace());
        }
    }

    public void slideUp(WaitLinear lp) throws InterruptedException {
        if(level == 6)
            robot.vSlides.moveTo4();
        else
            robot.vSlides.moveTo((int)(c1+(interval*(level-1))));

        /*if(signalColors == SignalColors.Blue.C){ robot.armAutoOut();}
        //robot.cupLocked();
        if(cycleNumber==0) {
            double level;
            if (signalColors == DuckPositions.A) {
                level = startencoder + 3;
            } else if (signalColors == DuckPositions.B) {
                robot.slides.moveToMid();
                level = level2;
            } else {
                robot.slides.moveToTop(true);
                level = level3;
            }
            if(signalColors==DuckPositions.A || signalColors==DuckPositions.B) {
                robot.armAutoOutShared();
                robot.cupLocked();
            }
        } else{
            robot.armAutoOut();
            robot.slides.moveToTop(true);
        }*/
        /*while (robot.getSlidePosition() < level) {
            double distance = level - robot.getSlidePosition();
            double calcPower = Math.signum(distance)*(Math.abs(distance)/level*3);
            robot.slideOn(Math.abs(calcPower) > 0.35 ? calcPower : Math.signum(calcPower)*0.35);
        }
        robot.slideOff();*/
    }

    /*private void checkSwitchBroken() {
        if (((robot.slides.getCurrentPosition() - startencoder) > 200) && robot.slides.isSlideSwitchPressed()) {
            switchBroken = true;
            robot.ledRedBlink();
        } else {
            switchBroken = false;
            robot.ledRedOn(false);
        }
    }*/

    private boolean slideReachedBottom() {
        //if (switchBroken) return robot.slides.getCurrentPosition() <= startencoder;
        return robot.vSlides.switchSlideDown.isTouch() && robot.vSlides.getCurrentPosition() <= robot.vSlides.start;
    }

    public void slideDown(WaitLinear lp) throws InterruptedException {
        robot.clawOpen();
        while(!slideReachedBottom() && slideDownTime-System.currentTimeMillis()<1000){
            robot.vSlides.moveToBottom();
        }
        robot.vSlides.forcestop();
        robot.vSlides.reset(robot.vSlides.slideLeft);
        robot.vSlides.reset(robot.vSlides.slideRight);

        /*robot.armMid();
        robot.cupOpen();
        lp.waitMillis(300);
        while (!slideReachedBottom() && slideDownTime-System.currentTimeMillis()<1000) {
            //robot.slideDown();
            robot.slides.moveToBottom();
        }
        robot.slides.forcestop();
        robot.cupOpen();
        robot.armDown();
        robot.slides.setPower(0);*/
    }
}
package overcharged.components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import overcharged.pid.Controller;
import overcharged.util.PIDCalculator;

public class oldSlide {

    public DcMotorEx left;
    public DcMotorEx right;
    public Servo cap;
    //starting encoder reading
    private double start;
    //current position of the slide
    public double position = 0;
    //-1 if motor is reverse
    int sign = 1;

    PIDCalculator pidController;

    public VoltageSensor battery;

    public static double in = 1;
    public static double out = 0.92;
    public static double up = 0.6;

    public boolean isCapOut = false;

    int positionL;
    int positionR;

    public oldSlide(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotorEx.class, "slideL");
        right = hardwareMap.get(DcMotorEx.class, "slideR");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        //left direction == DcMotorSimple.Direction.REVERSE then sign = -1 else sign = 1
        sign = 1;
        start = left.getCurrentPosition();

        left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficients = new PIDFCoefficients(15,0,0,0);
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

        cap = hardwareMap.servo.get("cap");
        cap.setPosition(in);

        pidController = new PIDCalculator(0.2, 0, 0.1);

        positionL = left.getCurrentPosition();
        positionR = right.getCurrentPosition();
    }

    public void up(){
        left.setPower(1);
        right.setPower(1);
    }
    public void down() {
        left.setPower(-0.8);
        right.setPower(-0.8);
    }

    public void position(int pow){
        left.setTargetPosition(pow);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(1);

        right.setTargetPosition(pow);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setPower(1);
    }

    public void mid(){
        left.setPower(0.3);
        right.setPower(0.3);
    }

    public void on(double pwr){
//        positionL = left.getCurrentPosition() + getDrift(left);
//        positionR = right.getCurrentPosition() + getDrift(right);
//        left.setTargetPosition(positionL);
//        right.setTargetPosition(positionL);
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setPower(pwr);
        right.setPower(pwr);
    }

    public void stop(){
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void off(){
        left.setPower(0);
        right.setPower(0);
    }

    public int getDrift(DcMotorEx motor){
        return (int)(0.05 * motor.getVelocity());
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
    }

    public void setPIDFCoefficients(PIDFCoefficients pid){
        left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
        right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
    }

    private void update(){
        position = sign * (left.getCurrentPosition() - start);
    }

    public double getSlidePosition(){
        update();
        return position;
    }

    public void resetSlidePosition(){
        start = left.getCurrentPosition();
        update();
    }

    /*public void extendLevel3(boolean upLimit){
        double sign = upLimit ? 1 : -1;
        pidController.setTargetPosition(upLimit ? 660 : 0);
        if(pidController.getLastError() > 5){
            double avgTicks = pidController.update(left.getCurrentPosition());
            left.setPower(sign*avgTicks);
            right.setPower(sign*avgTicks);
        }
    }*/

    public void keep(){
        double powerL = pidController.getPID(positionL);
        double powerR = pidController.getPID(positionR);
        left.setTargetPosition(positionL);
        right.setTargetPosition(positionR);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(powerL);
        right.setPower(powerR);
    }

    public void setCapPos(double pos){
        cap.setPosition(pos);
    }
}

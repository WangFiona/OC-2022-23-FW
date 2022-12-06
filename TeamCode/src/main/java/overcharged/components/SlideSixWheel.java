package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class SlideSixWheel {

    public DcMotorEx slide;

    public VoltageSensor battery;

    public SlideSixWheel(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotorEx.class, "slide");

        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0,0,0));

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    public void up(){
        slide.setPower(1);
    }
    public void down() {slide.setPower(-1);}

    public void position(int pow){
        slide.setTargetPosition(pow);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
    }

    public void mid(){
        slide.setPower(0.3);
    }

    public void on(double pwr){
        slide.setPower(pwr);
    }

    public void stop(){slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

    public void off(){
        slide.setPower(0);
    }

    public double getVelocity(){
        return (Math.abs(slide.getVelocity()));
    }

    public void setPIDFCoefficients(double p, double i, double d, double f){
        slide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,f));
    }

    public void setPIDFCoefficients(PIDFCoefficients pid){
        slide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid);
    }
}
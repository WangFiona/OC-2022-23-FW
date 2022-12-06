package overcharged.components;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class DuckNoPID {

    public DcMotor duck;

    public DuckNoPID(HardwareMap hardwareMap){
        duck = hardwareMap.get(DcMotor.class, "duck");

        duck.setDirection(DcMotorSimple.Direction.FORWARD);
        duck.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void on(){
        duck.setPower(0.485);
    }

    public void mid(){
        duck.setPower(0.44);
    }

    public void on(double pwr){
        duck.setPower(pwr);
    }

    public void max(){
        duck.setPower(1);
    }

    public void off(){
        duck.setPower(0);
    }

}

package overcharged.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Encoder {
    DcMotor encoder;
    private double start;
    public double position = 0;

    int sign = 1;

    public static double TICK_TO_INCH = 242.5521332720485;

    public Encoder(String name, DcMotorSimple.Direction direction, HardwareMap hardwareMap){
        encoder = hardwareMap.dcMotor.get(name);
        if(direction == DcMotorSimple.Direction.REVERSE) sign = -1;
        start = encoder.getCurrentPosition();
    }

    private void update(){
        position = sign*(encoder.getCurrentPosition() - start);
    }

    public double getPosition(){
        update();
        return position;
    }

    public void resetPosition(){
        start = encoder.getCurrentPosition();
        update();
    }

    public double getRawPosition(){
        update();
        return position;
    }
}

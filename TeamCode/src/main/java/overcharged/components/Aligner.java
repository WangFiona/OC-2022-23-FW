package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Aligner {
    public OcServo aligner;
    public OcServo pivot;

    public static final float INIT = 23f;//17f;//25f;//21f;
    public static float OUT = 183f;//194f;//182f;//182f;
    public static float TELE_OUT = 183f;
    public static final float HALF = 154f;//150f;

    public static float LEFT = 40;
    public static float RIGHT = 208;

    public Aligner(HardwareMap hardwareMap) {
        aligner = new OcServo(hardwareMap, "aligner", INIT);
        pivot = new OcServo(hardwareMap, "pivot", LEFT);
    }

    public void setPosition(float pos){
        aligner.setPosition(pos);
    }

    public void autoValue() { OUT = 182f; }

    public void setInit() { aligner.setPosition(INIT); }

    public void setOut() { aligner.setPosition(OUT); }

    public void setTeleOut() { aligner.setPosition(TELE_OUT); }

    public void setHalf() { aligner.setPosition(HALF); }

    public void setLeft() { pivot.setPosition(LEFT);}

    public void setRight() { pivot.setPosition(RIGHT);}
}

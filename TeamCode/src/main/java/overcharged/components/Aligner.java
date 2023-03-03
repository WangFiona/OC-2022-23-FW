package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Aligner {
    public OcServo aligner;

    public static final float INIT = 25f;//21f;
    public static float OUT = 198f;//194f;//182f;//182f;
    public static final float HALF = 154f;//150f;

    public Aligner(HardwareMap hardwareMap) {
        aligner = new OcServo(hardwareMap, "aligner", INIT);
    }

    public void setPosition(float pos){
        aligner.setPosition(pos);
    }

    public void autoValue() { OUT = 182f; }

    public void setInit() { aligner.setPosition(INIT); }

    public void setOut() { aligner.setPosition(OUT); }

    public void setHalf() { aligner.setPosition(HALF); }
}

package overcharged.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Aligner {
    public OcServo aligner;

    public static final float INIT = 29f;
    public static final float OUT = 200f;
    public static final float HALF = 150f;

    public Aligner(HardwareMap hardwareMap) {
        aligner = new OcServo(hardwareMap, "aligner", INIT);
    }

    public void setPosition(float pos){
        aligner.setPosition(pos);
    }

    public void setInit() { aligner.setPosition(INIT); }

    public void setOut() { aligner.setPosition(OUT); }

    public void setHalf() { aligner.setPosition(HALF); }
}

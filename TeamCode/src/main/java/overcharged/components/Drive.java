package overcharged.components;

import overcharged.config.RobotConstants;
import overcharged.util.GradualBrake;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Common methods for robot drive
 */
public abstract class Drive {

    /** AndyMark NeveRest 40 & 20 motor specs */
    public final static int AM40_ENCODER_TICK_PER_REVOLUTION = 1120;
    public final static int AM20_ENCODER_TICK_PER_REVOLUTION = 560;
    //NeveRest Orbital 20 Gearmotor
    // Ticks Per Revolution: 537.6
    public final static double AMO20_ENCODER_TICK_PER_REVOLUTION = 537.6;
    public final static int AM10_ENCODER_TICK_PER_REVOLUTION = 280;
    public final static float AM40_ENCODER_TICK_PER_CM =
        (float)(AM40_ENCODER_TICK_PER_REVOLUTION /
            (Math.PI * RobotConstants.WHEEL_DIAMETER_CM));
    public final static float AM20_ENCODER_TICK_PER_CM =
            (float)(AM20_ENCODER_TICK_PER_REVOLUTION /
                    (Math.PI * RobotConstants.WHEEL_DIAMETER_CM));
    public final static float AM20_ENCODER_TICK_PER_INCH =
            (float)(AM20_ENCODER_TICK_PER_REVOLUTION /
                    (Math.PI * RobotConstants.WHEEL_DIAMETER_INCH)); //560/12.57=44.55
    public final static float AMO20_ENCODER_TICK_PER_CM =
            (float)(AMO20_ENCODER_TICK_PER_REVOLUTION /
                    (Math.PI * RobotConstants.WHEEL_DIAMETER_CM));
    public final static float AMO20_ENCODER_TICK_PER_INCH =
            (float)(AMO20_ENCODER_TICK_PER_REVOLUTION /
                    (Math.PI * RobotConstants.WHEEL_DIAMETER_INCH)); //537.6/12.57=42.768
    public final static float AM10_ENCODER_TICK_PER_CM =
            (float)(AM10_ENCODER_TICK_PER_REVOLUTION /
                    (Math.PI * RobotConstants.WHEEL_DIAMETER_CM));
    public final static float AM40_ENCODER_TICK_PER_INCH = AM40_ENCODER_TICK_PER_CM * 2.54f; //104.645
    public final static float AM10_ENCODER_TICK_PER_INCH = AM10_ENCODER_TICK_PER_CM * 2.54f; //104.645
    public final static int AM40_MOTOR_RPM = 129;
    public final static int AM20_MOTOR_RPM = 340;
    public final static float AM40_MILLISECOND_PER_TICK =
        (float) (60000.0f / ((float) AM40_ENCODER_TICK_PER_REVOLUTION * AM40_MOTOR_RPM));
    public final static float AM20_MILLISECOND_PER_TICK =
            (float) (60000.0f / ((float) AM20_ENCODER_TICK_PER_REVOLUTION * AM20_MOTOR_RPM));
    public final static int TETRIX_ENCODER_TICK_PER_REVOLUTION = 1440;
    public final static int TETRIX_MOTOR_RPM = 137;
    public final static float TETRIX_MILLISECOND_PER_TICK =
            (float) (60000.0f / ((float) TETRIX_ENCODER_TICK_PER_REVOLUTION * TETRIX_MOTOR_RPM));

    public final static int TIMEOUT_DEFAULT = -1;

    private final static double MOTOR_ENCODER_TICK_PER_INCH = AM20_ENCODER_TICK_PER_INCH;

    public final OcMotor driveLeftFront;
    public final OcMotor driveLeftBack;
    public final OcMotor driveRightFront;
    public final OcMotor driveRightBack;
    public final OcMotor[] motorsArray;

    public final GradualBrake leftGradualBrake;
    public final GradualBrake rightGradualBrake;

    public Drive(OcMotor driveLeftFront,
                 OcMotor driveLeftBack,
                 OcMotor driveRightFront,
                 OcMotor driveRightBack)
    {
        this.driveLeftFront = driveLeftFront;
        this.driveLeftBack = driveLeftBack;
        this.driveRightFront = driveRightFront;
        this.driveRightBack = driveRightBack;
        this.motorsArray = new OcMotor[] {driveLeftFront, driveLeftBack, driveRightFront, driveRightBack};
        this.leftGradualBrake = new GradualBrake(driveLeftFront,
                                                 driveLeftBack);
        this.rightGradualBrake = new GradualBrake(driveRightFront,
                                                  driveRightBack);
    }

    /**
     * convert inches to encoder ticks
     * @param inch inches to be converted
     * @return encoder ticks for the given inches
     */
    public static int inchToTick(float inch) {
        return (int)(inch * MOTOR_ENCODER_TICK_PER_INCH);
    }
    /*public static int inchToTick(float inch) {
        return (int)(inch * Encoder.TICK_TO_INCH);
    }*/
    /**
     * convert inches to encoder ticks
     * @param inch inches to be converted
     * @return encoder ticks for the given inches
     */
    public static int inchToTick(int inch) {
        return (int)(inch * MOTOR_ENCODER_TICK_PER_INCH);
    }

    /**
     * convert encoder ticks to inches
     * @param ticks encoder ticks to be converted
     * @return inches for the given encoder ticks
     */
 //   public static int tickToInch(int ticks) {
 //       return (int)(ticks / MOTOR_ENCODER_TICK_PER_INCH);
//    }

    public static float tickToInch(double ticks) {
        return (float) (ticks / MOTOR_ENCODER_TICK_PER_INCH);
    }

    /*public static double tickToInch(double ticks) {
        return (double) (ticks / Encoder.TICK_TO_INCH);
    }*/

    /**
     * reset the encoder values of the drive motors
     */
    public void resetPosition() {
        driveLeftFront.resetPosition();
        driveLeftBack.resetPosition();
        driveRightBack.resetPosition();
        driveRightFront.resetPosition();
    }
    /*public void resetPosition() {
        left.resetPosition();
        right.resetPosition();
    }*/

    /**
     * stop the drive motors
     */
    public void stop() {
        setPower(0f);
    }

    public void setTank(double y1, double y2){
        setLeftPower(y1);
        setRightPower(y2);
    }

    public void setLeftPower(double power){
        driveLeftFront.setPower(power);
        driveLeftBack.setPower(power);
    }

    public void setRightPower(double power){
        driveRightFront.setPower(power);
        driveRightBack.setPower(power);
    }

    /**
     * run the drive motors
     * @param p motor power
     */
    public void setPower(float p) {
        driveLeftFront.setPower(p);
        driveLeftBack.setPower(p);
        driveRightBack.setPower(p);
        driveRightFront.setPower(p);
    }

    /**
     * turn the robot at a specified power
     * @param pwrl motor power left
     * @param pwrr motor power right
     * @param turnType turning method to use
     */
    public abstract void setPower(double pwrl,
                                  double pwrr,
                                  TurnType turnType);

    /**
     * set drive motor powers for floats
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }
    /**
     * check if the drive is busy
     */
    public boolean isBusy() {
        boolean lf = driveLeftFront.isBusy();
        boolean lb = driveLeftBack.isBusy();
        boolean rb = driveRightBack.isBusy();
        boolean rf = driveRightFront.isBusy();

        return  lf || lb || rb || rf;
    }

    public boolean isMode (DcMotor.RunMode mode) {
        boolean lf = driveLeftFront.getMode() == mode;
        boolean lb = driveLeftBack.getMode() == mode;
        boolean rb = driveRightBack.getMode() == mode;
        boolean rf = driveRightFront.getMode() == mode;

        return  lf && lb && rb && rf;
    }

    /**
     * set motor regulation
     * @param mode motor regulation to set to
     */
    public void setMode(DcMotor.RunMode mode)
    {
        driveLeftFront.setMode(mode);
        driveLeftBack.setMode(mode);
        driveRightBack.setMode(mode);
        driveRightFront.setMode(mode);
    }

}
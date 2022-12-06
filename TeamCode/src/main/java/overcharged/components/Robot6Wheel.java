package overcharged.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;
import overcharged.odometry.Localization;

/**
 * Overcharged Team #12599
 * Robot definition for Six wheel robot
 */
public class Robot6Wheel {
    protected Telemetry telemetry;

    ///Drive components
    public final OcMotor driveLeftFront;
    public final OcMotor driveLeftBack;
    public final OcMotor driveRightFront;
    public final OcMotor driveRightBack;

    public Encoder left;
    public Encoder right;

    public DcMotor leftEncMotor;
    public DcMotor rightEncMotor;

    public final Drive drive;

    ///Robot Gyro sensor
    public OcGyro2 gyroSensor;
    //public final OcGyro gyroSensor1;

    //public NavxImu gyroSensor;
    public Intake intake;
    public CupSixWheel cup;
    public Duck duck;
    public SlideSixWheel slide;
    public Encoder slideencoder;
    public OcSwitch limitSwitch;

    /// Odometry
    public Localization odometryLocalization = null;

    public DistanceSensor sensorDistance;

    public final List<OcServo> servos = new ArrayList<>();
    public final List<OcSwitch> switchs = new ArrayList<>();

//    ///Led indicator components
//    private final OcLed ledYellow;
//    private final OcLed ledGreen;
//    private final OcLed ledWhite;
//    private final OcLed ledBlue;
//    private final OcLed ledRed;
//    public final List<OcLed> leds = new ArrayList<>();

    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public Robot6Wheel(OpMode op,
                       boolean isAutonomous)
    {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        RobotLog.ii(RobotConstants.TAG_R, "Initializing motors");
        ///Initialize Motors
        OcMotor driveLeftFront = null;
        try {
            driveLeftFront = new OcMotor(hardwareMap,
                    "driveLF",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: driveLF " + e.getMessage());
            missing = missing + "driveLF";
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        OcMotor driveLeftBack = null;
        try {
            driveLeftBack = new OcMotor(hardwareMap,
                    "driveLB",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: driveLB " + e.getMessage());
            missing = missing + ", driveLB";
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        OcMotor driveRightFront = null;
        try {
            driveRightFront = new OcMotor(hardwareMap,
                    "driveRF",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: driveRF " + e.getMessage());
            missing = missing + ", driveRF";
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        OcMotor driveRightBack = null;
        try {
            driveRightBack = new OcMotor(hardwareMap,
                    "driveRB",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: driveRB " + e.getMessage());
            missing = missing + ", driveRB";
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        try {
            left = new Encoder("intakeL", DcMotorSimple.Direction.FORWARD, hardwareMap);
            right = new Encoder("intakeR", DcMotorSimple.Direction.REVERSE, hardwareMap);
        } catch(Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: encoders " + e.getMessage());
            numberMissing++;
        }

        // intake initialization
        try {
            intake = new Intake(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: intake " + e.getMessage());
            numberMissing++;
        }

        // duck initialization
        try {
            duck = new Duck(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: duck " + e.getMessage());
            numberMissing++;
        }

        DistanceSensor distanceSensor = null;

        OcGyro2 gyro2 = null;

        try {
            gyro2 = new OcBnoGyro2(hardwareMap, "limu", "rimu");
            while (isAutonomous && gyro2.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R, "missing: gyro_sensor " + e.getMessage());
            missing = missing + ", Gyro";
            numberMissing++;
        }

        this.gyroSensor = gyro2;

        try {
            gyroSensor = new OcBnoGyro2(hardwareMap, "limu", "rimu");
            while (gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R, "missing: navx " + e.getMessage());
            numberMissing++;
        }

        this.drive = createDrive();

        try {
            slide = new SlideSixWheel(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slide " + e.getMessage());
            numberMissing++;
        }

        try {
            slideencoder = new Encoder("slide", DcMotorSimple.Direction.FORWARD, hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slideencoder " + e.getMessage());
            numberMissing++;
        }

        try {
            cup = new CupSixWheel(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: cup " + e.getMessage());
            numberMissing++;
        }

        /*try {
            limitSwitch = new OcSwitch(hardwareMap,"limitSwitch", true);
            boolean isNull= limitSwitch==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(try)? " + isNull);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: limitSwitch " + e.getMessage());
            numberMissing++;
            boolean isNull= limitSwitch==null ? true : false;
            RobotLog.ii(TAG_H, "limitSwitch(catch)? " + isNull);
        }*/

        RobotLog.ii(RobotConstants.TAG_R,  "Initializing done");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

    public void intakeOn(){
        intake.on();
    }
    public void intakeOff(){
        intake.off();
    }
    public void intakeOut(){
        intake.out();
    }

    public void slideUp(){ slide.up();}
    public void slideDown(){ slide.down();}
    public void slideOn(double power){ slide.on(power);}
    public void slideMid(){ slide.mid();}
    public void slideOff(){ slide.off();}
    public void slideBrake(){slide.stop();}
    public void slidePosition(int pow){slide.position(pow);}

    public void duckOn(){ duck.on();}
    public void duckOn(double power){ duck.on(power);}
    public void duckMid(){ duck.mid();}
    public void duckMax(){duck.max();}
    public void duckOff(){ duck.off();}

    public void cupOpen(){cup.setOpen();}
    public void cupDump(){cup.setDump();}
    public void cupLocked(){cup.setLocked();}


    //public boolean isPressed(){return limitSwitch.isTouch();}

    public void getPosition(){slideencoder.getPosition();}

    /**
     * Robot and sensor shut down
     */
    public void close ()
    {
        if (this.drive != null) {
            this.drive.stop();
        } else {
            if (this.driveLeftFront != null) this.driveLeftFront.setPower(0f);
            if (this.driveLeftBack != null) this.driveLeftBack.setPower(0f);
            if (this.driveRightFront != null) this.driveRightFront.setPower(0f);
            if (this.driveRightBack != null) this.driveRightBack.setPower(0f);
        }

    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Drive createDrive () {
        return new TankDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }

    /**
     * initialize tank drive
     * @return TankDrive
     */
    public TankDrive getTankDrive () {
        return (TankDrive) this.drive;
    }

}
package overcharged.components;

import static overcharged.config.RobotConstants.TAG_R;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
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
 * Robot definition for Mecanum wheel robot
 */
public class oldRobotMecanum {
    protected Telemetry telemetry;

    ///Drive components
    public OcMotor driveLeftFront;
    public OcMotor driveLeftBack;
    public OcMotor driveRightFront;
    public OcMotor driveRightBack;

    public MecanumDrive drive;

    ///Led indicator components
    private final OcLed ledYellow;
    private final OcLed ledGreen;
    private final OcLed ledWhite;
    private final OcLed ledBlue;
    private final OcLed ledRed;
    public final List<OcLed> leds = new ArrayList<>();
//    public QwiicLEDStrip ledStrip;

    ///Robot Gyro sensor
    public OcBnoGyro2 gyroSensor;
    public Intake intake;
    public Arm arm;
    public Cup cup;
    public Duck duck;
    public Cap cap;
    public DuckNoPID duckNoPID;
    public oldSlide slide;
    public oldSlides slides;

    /// Odometry
    public Localization odometryLocalization = null;

    public DistanceSensor sensorDistance;
    public SensorDistance sensorDistanceR = null;
    public SensorDistance sensorDistanceL = null;
    public SensorDistance sensorDistanceF = null;

    public AnalogInput sonar = null;
    public AnalogInput sonarR = null;
    public AnalogInput sonarL = null;


    //public

    public final List<OcServo> servos = new ArrayList<>();


    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     * @param roadrunner if using roadrunner. If true, motors won't be initialized
     */
    public oldRobotMecanum(OpMode op,
                           boolean isAutonomous, boolean roadrunner)
    {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();


        /**
         * Initialize all devices. Log if initialization was unsuccessful. Devices:
         * Sonar sensor, 4 drive motors, gyro sensor, intake component,
         * duck spinner component, slide component, left & right distance sensor
         * cup component, LEDs, arm component, cap component
         */
        try {
            sonar = hardwareMap.get(AnalogInput.class, "sonar");
        } catch (Exception e) {
            RobotLog.e("missing: sonar " + e.getMessage());
        }
        try {
            sonarR = hardwareMap.get(AnalogInput.class, "sonarR");
        } catch (Exception e) {
            RobotLog.e("missing: sonarR " + e.getMessage());
        }
        try {
            sonarL = hardwareMap.get(AnalogInput.class, "sonarL");
        } catch (Exception e) {
            RobotLog.e("missing: sonarL " + e.getMessage());
        }
        if(!roadrunner){
            RobotLog.ii(RobotConstants.TAG_R, "Initializing motors");
            ///Initialize Motors
            OcMotor driveLeftFront = null;
            try {
                driveLeftFront = new OcMotor(hardwareMap,
                        "driveLF",
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                        DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (Exception e) {
                RobotLog.ee(RobotConstants.TAG_R,  "missing: driveRB " + e.getMessage());
                missing = missing + ", driveRB";
                numberMissing++;
            }
            this.driveRightBack = driveRightBack;

            this.drive = createDrive();
        }

        if (isAutonomous) {
            OcBnoGyro2 gyro2 = null;
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
        }

        // intake initialization
        try {
            intake = new Intake(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: claw " + e.getMessage());
            missing = missing + ", Claw";
            numberMissing++;
        }

        // duck initialization
        try {
            duck = new Duck(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: duck " + e.getMessage());
            missing = missing + ", Duck";
            numberMissing++;
        }

        try {
            duckNoPID = new DuckNoPID(hardwareMap);
        } catch (Exception e){
            RobotLog.ee(RobotConstants.TAG_R,  "missing: duck " + e.getMessage());
            missing = missing + ", Duck";
            numberMissing++;
        }

        try {
            slide = new oldSlide(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slide " + e.getMessage());
            missing = missing + ", Slide";
            numberMissing++;
        }

        try {
            slides = new oldSlides(this, hardwareMap, arm);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: slides " + e.getMessage());
            missing = missing + ", Slides";
            numberMissing++;
        }

        try {
            sensorDistanceF = new SensorDistance(hardwareMap, "rangeF");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceF" + e.getMessage());
            numberMissing++;
        }

        try {
            sensorDistanceR = new SensorDistance(hardwareMap, "rangeR");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceR" + e.getMessage());
            numberMissing++;
        }

        try {
            sensorDistanceL = new SensorDistance(hardwareMap, "rangeL");
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: distanceL" + e.getMessage());
            numberMissing++;
        }

        try {
            cup = new Cup(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: cup " + e.getMessage());
            missing = missing + ", Cup";
            numberMissing++;
        }

        try {
            arm = new Arm(hardwareMap, isAutonomous);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: arm " + e.getMessage());
            missing = missing + ", arm";
            numberMissing++;
        }

        try {
            cap = new Cap(hardwareMap);
        } catch (Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: cap " + e.getMessage());
            missing = missing + ", cap";
            numberMissing++;
        }

        ///Initialize Leds
        RobotLog.ii(TAG_R,  "Initializing Leds");
        OcLed led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_yellow");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_yellow " + e.getMessage());
            missing = missing + ", led_yellow";
            numberMissing++;
        }
        ledYellow = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_green");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_green " + e.getMessage());
            missing = missing + ", led_green";
            numberMissing++;
        }
        ledGreen = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_white");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_white " + e.getMessage());
            missing = missing + ", led_white";
            numberMissing++;
        }
        ledWhite = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_blue");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_blue " + e.getMessage());
            missing = missing + ", led_blue";
            numberMissing++;
        }
        ledBlue = led;
        led = null;
        try {
            led = new OcLed(hardwareMap,
                    "led_red");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_red " + e.getMessage());
            missing = missing + ", led_red";
            numberMissing++;
        }
        ledRed = led;

//        QwiicLEDStrip led = null;
//        try {
//            led = hardwareMap.get(QwiicLEDStrip.class, "ledstrip");
//            led.setBrightness(8);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: ledstrip " + e.getMessage());
//            missing = missing + ", led";
//            numberMissing++;
//        }
//        this.ledStrip = led;

        RobotLog.ii(RobotConstants.TAG_R,  "Initializing RobotMecanum complete");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

    /**
     * Robot and sensor shutdown
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
     * Initialize the underlying drive motor class (only for non-Roadrunner)
     * @return Drive
     */
    protected MecanumDrive createDrive () {
        return new MecanumDrive(
                driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack
        );
    }

    /**
     * return the Mecanum tank drive instance
     * @return MecanumDrive
     */
    public MecanumDrive getDrive () {
        return (MecanumDrive) this.drive;
    }

    /**
     * Proxy methods for all components
     */
    public void intakeOn(){
        intake.on();
    }
    public void intakeOff(){
        intake.off();
    }
    public void outtake(){
        intake.out();
    }
    public void duckOn(double power){ duck.on(power);}
    public void duckOff(){ duck.off();}


    public void cupOpen(){cup.setOpen();}
    public void cupDump(){cup.setDump();}
    public void cupLocked(){cup.setLocked();}
    public void cupDuckLocked(){cup.setDuckLocked();}

    public void armDown(){arm.setDown();}
    public void armMid(){arm.setMid();}
    public void armOut(){arm.setOut();}
    public void armOutShared(){arm.setOutShared();}
    public void armOutSharedReach(){ arm.setOutSharedReach(); }
    public void armOutSharedReachLess(){ arm.setOutSharedReachLess(); }

    public boolean isArmOut(){ return arm.armOut; }
    public boolean isArmDown(){ return arm.armDown; }
    public void armAutoOut() {arm.setAutoOut();}
    public void armAutoOutShared() {arm.setAutoShared();}


    public boolean isCollected(){ return cup.isCollected(); }
    public Cup.FreightType getFreight(){ return cup.getFreight(); }
    

    /**
     * Get the distance from the front of the robot to the front wall.
     * This works in three steps. Initially, the raw sonar voltage is read from the sensor.
     * Second, the number is converted to a distance as per the datasheet.
     * Lastly, the value is tuned based on a line of best fit
     * @return
     */
    public double getSonar(){
        float sonarDistance = (float) (((this.sonar.getVoltage() * 6f * 1024f / 2.7f) - 300f) / 25.4f);
        double distToWall = (float) (0.9829f * sonarDistance - 0.5991);
        return distToWall;
    }

    /**
     * Set the capping mechanism to the specified position
     * @param pos position value
     */
    public void setCapPos(float pos){ cap.setCapPos(pos); }

    /**
     * Set the capping mechanism to out position and ready for capping
     */
    public void setCapOut(){ cap.setCapOut(); }

    /**
     * Set the capping mechanism to in position
     */
    public void setCapIn(){ cap.setCapIn(); }

    /**
     * Set the capping mechanism to up position just before capping
     */
    public void setCapUp(){ cap.setCapUp(); }

    /**
     * Complete capping
     */
    public void setCapped(){ cap.setCapped(); }

    /**
     * update LEDs to display the colors
     */
    public void drawLed () {
        for (OcLed led: leds) {
            led.draw();
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledYellowOn(boolean on) {
        try {
            if (on) {
                this.ledYellow.on();
            } else {
                this.ledYellow.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledBlueOn(boolean on) {
        try {
            if (on) {
                this.ledBlue.on();
            } else {
                this.ledBlue.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if true turn on
     */
    public void ledGreenOn(boolean on) {
        try {
            if (on) {
                this.ledGreen.on();
            } else {
                this.ledGreen.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledGreenBlink() {
        try {
            this.ledGreen.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Green " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledBlueBlink() {
        try {
            this.ledBlue.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Blue " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledRedBlink() {
        try {
            this.ledRed.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Red " + e.getMessage());
        }
    }
    /**
     * Protect the led access and blink
     */
    public void ledWhiteBlink() {
        try {
            this.ledWhite.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_White " + e.getMessage());
        }
    }

    /**
     * Protect the led access and blink
     */
    public void ledYellowBlink() {
        try {
            this.ledYellow.blink();
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: blinking led_Yellow " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else false
     */
    public void ledRedOn(boolean on) {
        try {
            if (on) {
                this.ledRed.on();
            } else {
                this.ledRed.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_Red " + e.getMessage());
        }
    }

    /**
     * Protect the led access and turn on/off
     * @param on if tru turn on else false
     */
    public void ledWhiteOn(boolean on) {
        try {
            if (on) {
                this.ledWhite.on();
            } else {
                this.ledWhite.off();
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "Error: led_White " + e.getMessage());
        }
    }

}
package overcharged.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

import overcharged.config.RobotConstants;

import static overcharged.config.RobotConstants.SERVO_GRABBERL_IN;
import static overcharged.config.RobotConstants.SERVO_GRABBERR_IN;
import static overcharged.config.RobotConstants.TAG_R;

/**
 * Overcharged Team #12599
 * Robot definition for Six wheel test robot
 */
public class Robot6WheelTest2 {
    protected Telemetry telemetry;

    ///Drive components
    public final OcMotor driveLeftFront;
    public final OcMotor driveLeftBack;
    public final OcMotor driveRightFront;
    public final OcMotor driveRightBack;

    public Encoder left;
    public Encoder right;
//    ///Intake components
//    public final OcMotor intakeLeft;
//    public final OcMotor intakeRight;
//    ///Grabber components
//    public final OcServo servoGrabberL;
//    public final OcServo servoGrabberR;

    public final Drive drive;

    ///Robot Gyro sensor
    public final OcGyro gyroSensor;

//    ///Foundation grabber object
//    public final FoundationGrabber foundationGrabber;

    //public final List<OcServo> servos = new ArrayList<>();

    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public Robot6WheelTest2(OpMode op,
                           boolean isAutonomous)
    {
        String missing = "";
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        RobotLog.ii(TAG_R, "Initializing motors");
        ///Initialize Motors
        OcMotor driveLeftFront = null;
        try {
            driveLeftFront = new OcMotor(hardwareMap,
                    "driveLF",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: driveLF " + e.getMessage());
            missing = missing + "driveLF";
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        OcMotor driveLeftBack = null;
        try {
            driveLeftBack = new OcMotor(hardwareMap,
                    "driveLB",
                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: driveLB " + e.getMessage());
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
            RobotLog.ee(TAG_R,  "missing: driveRF " + e.getMessage());
            missing = missing + ", driveRF";
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        OcMotor driveRightBack = null;
        try {
            driveRightBack = new OcMotor(hardwareMap,
                    "driveRB",
                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: driveRB " + e.getMessage());
            missing = missing + ", driveRB";
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        try {
            left = new Encoder("driveLB", DcMotorSimple.Direction.FORWARD, hardwareMap);
            right = new Encoder("driveRF", DcMotorSimple.Direction.REVERSE, hardwareMap);
        } catch(Exception e) {
            RobotLog.ee(RobotConstants.TAG_R,  "missing: encoders " + e.getMessage());
            numberMissing++;
        }

        RobotLog.ii(TAG_R, "Initializing Gyro sensor");
        ///Initialize the Gyro sensor used in this robot
        OcGyro gyro = null;
        try {
            gyro = new OcBnoGyro(hardwareMap, "imu");
            //gyroSensor = new OcBnoGyro(hardwareMap, "imu");
            //gyroSensor = new OcNavGyro(hardwareMap, "imu2");
            while (isAutonomous && gyro.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: gyro_sensor " + e.getMessage());
            missing = missing + ", Gyro";
            numberMissing++;
        }
        this.gyroSensor = gyro;
        this.drive = createDrive();

//        OcMotor intakeL = null;
//        try {
//            intakeL = new OcMotor(hardwareMap,
//                    "intakeL",
//                    DcMotor.Direction.FORWARD, DcMotor.RunMode.RUN_USING_ENCODER);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: intakeL " + e.getMessage());
//            missing = missing + ", intakeL";
//            numberMissing++;
//        }
//        this.intakeLeft = intakeL;
//
//        OcMotor intakeR = null;
//        try {
//            intakeR = new OcMotor(hardwareMap,
//                    "intakeR",
//                    DcMotor.Direction.REVERSE, DcMotor.RunMode.RUN_USING_ENCODER);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: intakeR " + e.getMessage());
//            missing = missing + ", intakeR";
//            numberMissing++;
//        }
//        this.intakeRight = intakeR;
//
//        OcServo ocServo = null;
//        try {
//            ocServo = new OcServo(hardwareMap, "servoGrabberL", SERVO_GRABBERL_IN);
//            servos.add(ocServo);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: servoGrabberL " + e.getMessage());
//            missing = missing + ", servoGrabberL";
//            numberMissing++;
//        }
//        this.servoGrabberL = ocServo;
//
//        ocServo = null;
//        try {
//            ocServo = new OcServo(hardwareMap, "servoGrabberR", SERVO_GRABBERR_IN);
//            servos.add(ocServo);
//        } catch (Exception e) {
//            RobotLog.ee(TAG_R,  "missing: servoGrabberR " + e.getMessage());
//            missing = missing + ", servoGrabberR";
//            numberMissing++;
//        }
//        this.servoGrabberR = ocServo;
//
//        foundationGrabber = new FoundationGrabber(this.servoGrabberL, this.servoGrabberR);

        RobotLog.ii(TAG_R,  "Initializing done");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.addData("Missing", missing);
        telemetry.update();
    }

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
     * initialize swerve drive
     * @return TankDrive
     */
    public TankDrive getTankDrive () {
        return (TankDrive) this.drive;
    }
}
package overcharged.config;

/**
 * Shared robot constants (i.e. switch addresses, servo presets, etc)
 */
public interface RobotConstants {

    ///Overcharged TAG for Stacker class
    String TAG_STKR = "Overcharged Stacker::";

    class COLOR_SENSOR {
        public int channel;
        public COLOR_SENSOR(int i) { channel = i; }
    }

    /** AndyMark stealth wheel */
    float WHEEL_DIAMETER_CM = 10.16f;
    /** AndyMark stealth wheel inches*/
    int WHEEL_DIAMETER_INCH = 4;
    ///Overcharged TAG for log parsing
    String TAG_R = "Overcharged Robot::";
    ///Overcharged TAG for device og parsing
    String TAG_Device = "Overcharged Device::";
    ///Overcharged TAG for Autonomous log parsing
    String TAG_A = "Overcharged Auto::";
    ///Overcharged TAG for Teleop log parsing
    String TAG_T = "Overcharged TeleOp::";
    ///Overcharged TAG for Slide log parsing
    String TAG_SL = "Overcharged Slide::";
    ///Overcharged TAG for Swerve log parsing
    String TAG_SW = "Overcharged Swerve::";
    ///Overcharged TAG for Diagnostics log parsing
    String TAG_D = "Overcharged Diag::";
    ///Overcharged TAG for Setup log parsing
    String TAG_S = "Overcharged Setup::";
    ///Overcharged TAG for Vuforia log parsing
    String TAG_V = "Overcharged Vuforia::";
    ///Overcharged TAG for Odometry log parsing
    String TAG_O = "Overcharged Odometry::";
    ///Overcharged TAG for Hardware logs
    String TAG_H = "Overcharged Hardware::";
    ///Overcharged TAG for Tank drive log parsing
    String TAG_TDRV= "Overcharged TankDrive::";
    ///Overcharged TAG for Tank drive log parsing
    String TAG_TD= "Overcharged TankDriveLinear::";
    ///Overcharged TAG for 4 Bar log parsing
    String TAG_4B= "Overcharged 4 Bar::";
    ///Overcharged TAG for Skystone log parsing
    String TAG_SK = "Overcharged Skystone::";
    ///Overcharged TAG for Skystone log parsing
    String TAG_PRSTS = "Overcharged Persist::";

    ///Foundation grabber servo values
    //The servos have different values for the same position so the left and right difference cannot be the same using 93 for left and 8 for right
//    float SERVO_GRABBERL_LATCH = 139f;
//    float SERVO_GRABBERL_IN = 58f;
//    float SERVO_GRABBERL_READY_FOR_LATCH = (SERVO_GRABBERL_IN + SERVO_GRABBERL_LATCH)/2f;
//    float SERVO_GRABBERR_LATCH = 112f;
//    float SERVO_GRABBERR_IN = 189f;
//    float SERVO_GRABBERR_READY_FOR_LATCH = (SERVO_GRABBERR_IN + SERVO_GRABBERR_LATCH)/2f;
    float SERVO_GRABBERL_IN = 35f;
    float SERVO_GRABBERL_READY_FOR_LATCH = 158f;
    float SERVO_GRABBERL_LATCH = 193f;
    float SERVO_GRABBERR_IN = 231f;
    float SERVO_GRABBERR_READY_FOR_LATCH = 100f;
    float SERVO_GRABBERR_LATCH = 70f;

    ///4 bar claw
    float SERVO_CLAW_CLOSE = 88f;
    float SERVO_CLAW_OPEN = 219f;

    /// Here is how you calibrate the four bar
    /// First calibrate the parallel position to the slide
    /// Left is 86
    /// Right is 173
    /// So difference is 87
    /// Now calculate the down position of right, which is 108
    /// the difference from 173 to 108 is 65, then add 65 to 86 to get fourBarDownL which is 151

    ///This is the only value to calibrate, servo starting value when its inside the robot and ready to grab.
    ///The rest is auto calculated. This is very useful during servo replacement/calibration
    //Servo fail safe  values for Left 1741 1948
    //Servo fail safe  values for Right 1280 1073

    ///virtual 4 bar servo starting value when its inside the robot and ready to grab.
    float SERVO_FOURBARL_DOWN = 184f; //left level1 77 down 171 turn 94. Adding will bring the bar down
    float SERVO_FOURBARR_DOWN = 72f; //right down=81-85 level1-178. Subtracting will bring the bar down + used to be 59
    float fourBarDiff = (SERVO_FOURBARL_DOWN - SERVO_FOURBARR_DOWN) * 255f;
    ///virtual 4 bar servo starting value when its inside the robot and waiting for intake to bring the stone in.
    float SERVO_FOURBARL_UP = 169f;
    float SERVO_FOURBARR_UP = 87f;
    ///virtual 4 bar servo value when the slides move up for alignment.
    float SERVO_FOURBARL_SLIDEUP = 170f;
    float SERVO_FOURBARR_SLIDEUP = 86f;
    ///virtual 4 bar servo starting value when its outside the robot and in position to place the stone at level2.
//    float SERVO_FOURBARL_OUT = 92f;
//    float SERVO_FOURBARR_OUT = 158f;
    float SERVO_FOURBARL_OUT = 111f;
    float SERVO_FOURBARR_OUT = 142f;
    ///virtual 4 bar servo starting value when its outside the robot and in position to place the stone at level1.
    float SERVO_FOURBARL_LEVEL1 = 99f;
    float SERVO_FOURBARR_LEVEL1 = 153f;

    ///Four bar Stacker top rotation servo
    ///Value returned by the tester calibration. Only set the STACKER_START value. The rest of them are calculated.
    ///This is the stacker top servo starting value when its inside the robot. Start to horizontal goes clockwise
    float STACKER_START= 123f;
    ///Stacker top servo starting value when its inside the robot. Start to horizontal goes clockwise
    float SERVO_STACKER_START = STACKER_START;
    ///Stacker top servo value when its outside and 90 degrees from the starting position
    float SERVO_STACKER_HORIZONTAL = (STACKER_START - 76f);

    ///2019-20120 Constants league meet 0
    //float SERVO_DUMPER_DOWN = 244f;
    //float SERVO_DUMPER_UP = 205f;
    //float SERVO_DUMPER_DUMP = 107f;

    ///Left side rotate/claw used for autonomous. This is on the robot's left side
    ///Only Calibrate the servoLsideRotateDown value. Normally it matches with the right side rotate constants
    float servoLsideRotateInit = 40f;
    float SERVO_LSIDE_CLAW_ROTATE_INIT = servoLsideRotateInit;
    float SERVO_LSIDE_CLAW_ROTATE_PREP = (servoLsideRotateInit + 6);
    float SERVO_LSIDE_CLAW_ROTATE_UP = (servoLsideRotateInit + 20f);
    float SERVO_LSIDE_CLAW_ROTATE_DUMP = (servoLsideRotateInit + 130f);
    float SERVO_LSIDE_CLAW_ROTATE_DUMP2 = (servoLsideRotateInit + 125f);
    float SERVO_LSIDE_CLAW_ROTATE_DUMP3 = (servoLsideRotateInit + 106f);
    float SERVO_LSIDE_CLAW_ROTATE_DOWN = (servoLsideRotateInit + 158f);

    ///Left side claw part
    float servoLsideClawInit = 19f;
    float SERVO_LSIDE_CLAW_INIT = servoLsideClawInit;
    float SERVO_LSIDE_CLAW_CLOSE = (servoLsideClawInit + 3f);
    float SERVO_LSIDE_CLAW_OPEN2 = (servoLsideClawInit + 27f);
    float SERVO_LSIDE_CLAW_OPEN3 = (servoLsideClawInit + 75f);
    float SERVO_LSIDE_CLAW_OPEN = (servoLsideClawInit + 80f);

    ///Right side rotate/claw used for autonomous. This is on the robot's right side
    ///Only Calibrate the servoRsideRotateDown value
    float servoRsideRotateInit = 196f;
    float SERVO_RSIDE_CLAW_ROTATE_INIT = servoRsideRotateInit;
    float SERVO_RSIDE_CLAW_ROTATE_PREP = (servoRsideRotateInit - 5f);
    float SERVO_RSIDE_CLAW_ROTATE_UP = (servoRsideRotateInit - 25f);
    float SERVO_RSIDE_CLAW_ROTATE_DUMP = (servoRsideRotateInit - 130f);
    float SERVO_RSIDE_CLAW_ROTATE_DUMP2 = (servoRsideRotateInit - 125f);
    float SERVO_RSIDE_CLAW_ROTATE_DUMP3 = (servoRsideRotateInit - 106f);
    float SERVO_RSIDE_CLAW_ROTATE_DOWN = (servoRsideRotateInit - 158f);

    ///Right side claw part
    float servoRsideClawInit = 125f;
    float SERVO_RSIDE_CLAW_INIT = servoRsideClawInit;
    float SERVO_RSIDE_CLAW_CLOSE = (servoRsideClawInit - 7f);
    float SERVO_RSIDE_CLAW_OPEN2 = (servoRsideClawInit - 27f);
    float SERVO_RSIDE_CLAW_OPEN3 = (servoRsideClawInit - 75f);
    float SERVO_RSIDE_CLAW_OPEN = (servoRsideClawInit - 80f);

    ///Capstone
    float SERVO_CAPSTONE_TELEOP_START = 158f;
    float SERVO_CAPSTONE_AUTO = 159f;
    float SERVO_CAPSTONE_TELEOP_DELIVER = 194f;
    float SERVO_CAPSTONE_TELEOP_AFTER_DELIVER = 187f;

    ///Parking
    float SERVO_PARKING_STOP = 127f;
    float SERVO_PARKING_RELEASE = 210f;
}
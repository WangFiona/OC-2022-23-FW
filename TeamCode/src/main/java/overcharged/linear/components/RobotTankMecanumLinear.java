package overcharged.linear.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import overcharged.components.Drive;
import overcharged.components.RobotTankMecanum;

/**
 * This class is for Autonomous
 */

public class RobotTankMecanumLinear
        extends RobotTankMecanum
{
    protected LinearOpMode op;

    public RobotTankMecanumLinear(LinearOpMode op) // if lazy
    {
        this(op, false);
    }

    public RobotTankMecanumLinear(LinearOpMode op, boolean lazy) // if lazy
    {
        super(op,true, lazy);
        this.op = op;
        if(!lazy){
            getTankDriveLinear().setLinearOpMode(op);
        }
    }



    @Override
    protected Drive createDrive() {
        return new TankDriveLinear(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                gyroSensor,
                odometryLocalization);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }

}

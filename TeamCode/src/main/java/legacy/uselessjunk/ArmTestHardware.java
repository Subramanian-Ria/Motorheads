//THIS IS A MOTORHEADS PROGRAM

package legacy.uselessjunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//@Disabled


public class ArmTestHardware
{
    /* Public OpMode members. */
    //Initialize everything
    public DcMotor armEx;
    public DcMotor armFlip;
    public  DcMotor intake;

    public DcMotor elevator;


    //declaring values for use with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.19685039;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     VERT_INCHES             = 3.0;      //inches to raise verticalarm       TODO: test for real value
    static final double     HORZ_INCHES             = 3.0;      //inches to extend horzizantalarm   TODO: test for real value
    static final double     EN_ARM_SPEED            = 0.2;      //speed of arm movement

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        armEx = hwMap.get(DcMotor.class, "armEx");
        armFlip = hwMap.get(DcMotor.class, "armFlip");
        intake = hwMap.get(DcMotor.class, "intake");

        elevator = hwMap.get(DcMotor.class, "ele");

        //Setting Motor Directions
        //fLMotor.setDirection(DcMotor.Direction.FORWARD); //right and left should be correct
        //fRMotor.setDirection(DcMotor.Direction.FORWARD);
        //bLMotor.setDirection(DcMotor.Direction.REVERSE);
        //bRMotor.setDirection(DcMotor.Direction.FORWARD);

        armEx.setPower(0);
        armFlip.setPower(0);
        intake.setPower(0);

        elevator.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        armEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setting 0 power behavior to brake
        armEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set directions
        armEx.setDirection(DcMotor.Direction.FORWARD);
        armFlip.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        elevator.setDirection(DcMotor.Direction.FORWARD);
    }
}
package legacy.uselessjunk;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by WillieShi on 10/2/2017.
 */

@Autonomous(name = "GyroDriveTest", group = "GyroTest")
//@Disabled
public class GyroDriveTest extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwarePushbot2 robot   = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .4;
    static final double TURN_SPEED = .2;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public float NORTH;
    public float EAST;
    public float WEST;
    public float SOUTH;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gparameters.loggingEnabled      = true;
        Gparameters.loggingTag          = "IMU";
        Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);

        //Vuforia Message storage do know what glpyh to put it in
        String glypos = "not visible";

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        //More Vuforia Setup
         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information
         * */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        updateAngles();


        //grasp the particle and lift it slightly

        //Vuforia
        runtime.reset();

        //Jewel Capture Code
        encoderDrive(48, 48, 10, .6, angles.firstAngle, 10.0);

        //keep in mind that the robot reads the jewel behind it

            /*
            sleep(100);
            //help to get the bot off the plat
            robot.motorLeft.setPower(-1);
            sleep(200);
            turnToNorth();
            sleep(100);
            turnDegrees(-90, TURN_SPEED, 5);
            sleep(100);
            encoderDrive(0, 0, -2, 5, DRIVE_SPEED);
            */
            //encoderDrive(4, 4, 0, 5, DRIVE_SPEED);




    }

    public void turnToNorthAlt() //just rename to turnToNorth() to change
    {
        updateAngles();
        double currentAngle = angles.firstAngle;
        double turnDir = NORTH - currentAngle;
        while ((currentAngle >= NORTH + 5 && currentAngle <= NORTH - 5) && opModeIsActive() && runtime.seconds() < 5)
        {
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (turnDir >= 0) //if the angle resulting from above is positive that means that current angle is likely more than NORth which means you wanna turn left
            {
                robot.leftDrive.setPower(.25);
                robot.rightDrive.setPower(-.25);
            }
            else
            {
                robot.leftDrive.setPower(-.25);
                robot.rightDrive.setPower(.25);
            }
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    public void turnToNorth() //just rename to turnToNorthAlt() to change //logic checked out with Albert, testing now
    {
        updateAngles();
        double currentAngle = angles.firstAngle;
        double target = currentAngle - NORTH; //it could be the other way around...
        turnDegrees(target, TURN_SPEED, 5);
    }

    public void turnDegrees(double target, double power, double timeoutS) //logic checked out with Albert -- testing right now
    {
        //Write code to correct to a target position (NOT FINISHED)
        runtime.reset();
        updateAngles(); //variable for gyro correction around z axis
        double error = angles.firstAngle - target;
        double errorAbs = Math.abs(error);

        //wrapping error to have it remain in the field
        if (error > 180)  error -= 360;
        if (error <= -180) error += 360;

        double powerScaled = power;
        do
        {
            updateAngles();
            error = angles.firstAngle - target;
            errorAbs = Math.abs(error);

            if (errorAbs <= 10)
            {
                powerScaled /= 2;
            }
            telemetry.addData("error", error);
            telemetry.update();
            if(error > 0)
            {
                robot.leftDrive.setPower(powerScaled);
                robot.rightDrive.setPower(-powerScaled);
            }
            else if(error < 0)
            {
                robot.leftDrive.setPower(-powerScaled);
                robot.rightDrive.setPower(powerScaled);
            }
        }
        while ((errorAbs > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }

    //run this everytime you need to update a direction variable
    public void updateAngles()
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double leftInches, double rightInches, double timeoutS, double Speed, double refAngle, double angleRange)
    {
        updateAngles();
        double currentAngle = angles.firstAngle;
        int newLeftTarget;
        int newRightTarget;
        double corrLeft = 0;
        double corrRight = 0;

        leftInches *= 1;
        rightInches *= 1;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(Speed));
            robot.rightDrive.setPower(Math.abs(Speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.addData("Speed", "Speed " + robot.leftDrive.getPower() + " " + robot.rightDrive.getPower());
                telemetry.addData("Current Angle", angles.firstAngle);
                telemetry.update();
                while(currentAngle != refAngle) {
                    corrLeft = ((currentAngle - refAngle) * 2)/angleRange;
                    corrRight = ((currentAngle - refAngle) * 2)/angleRange;
                    updateAngles();
                    currentAngle = angles.firstAngle;
                    robot.leftDrive.setPower(Speed + corrLeft);
                    robot.rightDrive.setPower(Speed - corrRight);
                }
                robot.leftDrive.setPower(Speed);
                robot.rightDrive.setPower(Speed);
                updateAngles();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}




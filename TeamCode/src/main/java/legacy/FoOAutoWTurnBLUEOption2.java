package legacy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by WillieShi on 10/2/2017.
 */

@Autonomous(name = "FoOAutonomousWTurnBLUEOP2", group = "Force of Haskins")
@Disabled
public class FoOAutoWTurnBLUEOption2 extends LinearOpMode
{
    /* Declare OpMode members. */
    ForceofHaskinsHardware robot   = new ForceofHaskinsHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .4;
    static final double TURN_SPEED = .2;

    //Vuforia Setup
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public float NORTH;

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

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motorLeft.getCurrentPosition(),
                robot.motorRight.getCurrentPosition());
        telemetry.update();

        //More Vuforia Setup
         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters VFparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VFparameters.vuforiaLicenseKey = "AXmdhMT/////AAAAGUaKcDAl00pauYqirixLh78YyL/PVKTvRqpancjUCke/xBIuz3p6PjSEmnZ8DShA1mjtzBSG6T9q6JvzM4ARnrGyi4pTQnVUGYgUs8HglThicDhm1xQRvQD3eckc2yjOfC591Bmq1hC4IOGgcMEsOCGPEwgTFKFoZh+c5z/P0Ta6H7S8k1X0Igx9DbLp1/DxU7nsxBKCmOyjsy1kcdB+6zbUiY3CQKqbLYjNEQWs7sStW8+lWtQZ/LA1JBpdXxuZ//68iAjhMVlaojy5nLG3QvcGUZRHSXa3fFyvOcHCYWLfqWzVrUawbj4zVkPQ4LrDdLHPrIviWE+YdgF4mWLvfPOxqzkqMMwmWnnKxMtKqpoG";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        VFparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(VFparameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //grasp the particle and lift it slightly
        robot.vaServoRight.setPosition(.6);
        robot.vaServoLeft.setPosition(.7);
        sleep(500);

        robot.verticalarm.setPower(1);
        sleep(500);

        robot.verticalarm.setPower(0);
        sleep(500);

        relicTrackables.activate();

        //Vuforia
        runtime.reset();
        while(glypos == "not visible" && runtime.seconds() <= 4 && opModeIsActive())
        {
            telemetry.addData("pos", glypos);
            telemetry.update();
            glypos = Vuforia(relicTemplate);
            if (runtime.seconds() >= 3) //if it doesn't read anything after 3 seconds, then just move one and assume right (b/c its the closest)
            {
                glypos = "RIGHT";
            }
        }

        //Jewel Capture Code
        double blueValue = 0;
        double redValue = 0;
        double postrack = .56;

        robot.touchArm.setPosition(.3); //setting it out of the way (0 is out so putting at .3 means its further out than in)
        sleep(100);
        robot.colorServo.setPosition(postrack); //used to be .8 before they geared up the servo

        sleep(500);
        for(int i = 0; i < 5; i++) //detecting jewel colors
        {
            redValue += robot.sensorColor.red();
            blueValue += robot.sensorColor.blue();
            postrack -= .005;
            robot.colorServo.setPosition(postrack);

            sleep(100);
        }
        blueValue/= 5;
        redValue /= 5;
        telemetry.addData("Blue", blueValue);
        telemetry.addData("Red: ", redValue);
        telemetry.update();
        sleep(100);

        //keep in mind that the robot reads the jewel behind it
        if(blueValue < redValue) // if blue then rotate
        {
            encoderDrive(-6, -6, 0, 10, 1);
            sleep(500);

            robot.colorServo.setPosition(1);
            sleep(1000);

            turnDegrees(-90, TURN_SPEED, 5);
            sleep(100);

            robot.touchArm.setPosition(0);
            sleep(100);

            encoderDrive(4, 4, 0, 5, DRIVE_SPEED);
        }
        else if(redValue < blueValue)
        {
            turnDegrees(180, TURN_SPEED, 10);
            sleep(100);

            robot.colorServo.setPosition(1);
            sleep(1000);

            encoderDrive(6, 6, 0, 10, 1); //get off the ramp
            sleep(100);

            turnDegrees(90, TURN_SPEED, 5);
            sleep(100);

            robot.touchArm.setPosition(0);
            sleep(100);

            encoderDrive(4, 4, 0, 5, DRIVE_SPEED);
            sleep(500);
        }

        if(glypos == "RIGHT")
        {
            /*
            encoderDrive(0, 0, -4, 2.0, DRIVE_SPEED);
            encoderDrive(-3.5, -3.5, 0, 2.0, DRIVE_SPEED);
            robot.vaServoLeft.setPosition(0);
            robot.vaServoRight.setPosition(1);
            */
            glyposAbs(glypos);
        }
        else if(glypos == "CENTER")
        {
            /*
            encoderDrive(0, 0 , -13, 1.0, DRIVE_SPEED);
            encoderDrive(-3.5, -3.5, 0, 2.0, DRIVE_SPEED);
            robot.vaServoLeft.setPosition(0);
            robot.vaServoRight.setPosition(1);
            */
            glyposAbs(glypos);
        }
        else if(glypos == "LEFT")
        {
            /*
            encoderDrive(0, 0 , -24, 1.0, DRIVE_SPEED);
            encoderDrive(-3.5, -3.5, 0, 2.0, DRIVE_SPEED);
            robot.vaServoLeft.setPosition(0);
            robot.vaServoRight.setPosition(1);
            */
            glyposAbs(glypos);
        }

        sleep(500);
        encoderDrive(-4.5, -4.5, 0, 5.0, DRIVE_SPEED);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void glyposAbs(String glypos)
    {
        if (glypos == "RIGHT") //needs to hit the column once before inserting glyph
        {
            moveAndCheck(3);
        }
        else if (glypos == "CENTER") //needs to hit the columns twice before inserting glyph
        {
            moveAndCheck(2);
        }
        else if (glypos == "LEFT")
        {
            moveAndCheck(1);
        }
        //asdf adsf asdf
    }

    public void moveAndCheck(int count)
    {
        for(int i = 0; i < count; i++)
        {
            //turnToNorth();
            encoderDrive(0, 0, .5, 2, .1);
            sleep(100);
            robot.touchArm.setPosition(0);
            while(robot.touchRed.getState() && opModeIsActive())
            {
                robot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorCenter.setPower(.2);
            }
            robot.motorCenter.setPower(0);
            robot.touchArm.setPosition(1);
            sleep(500);
        }
        encoderDrive(0, 0, 3, 5, DRIVE_SPEED);
        sleep(100);
        //letting go of glyph
        robot.vaServoLeft.setPosition(.2);
        robot.vaServoRight.setPosition(.8);
        sleep(100);
        encoderDrive(3., 3., 0., 5., DRIVE_SPEED);
    }

    public String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public String Vuforia(VuforiaTrackable relicTemplate)
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN)
        {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;


            }
            return vuMark.toString();
        }
        else
        {
            telemetry.addData("VuMark", "not visible");
            return "not visible";
        }
    }

    public void turnToNorthAlt() //just rename to turnToNorth() to change TODO: gotta double check this logic w/ Albert Yang
    {
        updateAngles();
        double currentAngle = angles.firstAngle;
        double turnDir = NORTH - currentAngle;
        while ((currentAngle >= NORTH + 5 && currentAngle <= NORTH - 5) && opModeIsActive() && runtime.seconds() < 5)
        {
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (turnDir >= 0) //if the angle resulting from above is positive that means that current angle is likely more than NORth which means you wanna turn left
            {
                robot.motorRight.setPower(.25);
                robot.motorLeft.setPower(-.25);
            }
            else
            {
                robot.motorRight.setPower(-.25);
                robot.motorLeft.setPower(.25);
            }
        }
        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
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
                robot.motorRight.setPower(powerScaled);
                robot.motorLeft.setPower(-powerScaled);
            }
            else if(error < 0)
            {
                robot.motorRight.setPower(-powerScaled);
                robot.motorLeft.setPower(powerScaled);
            }
        }
        while ((errorAbs > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

        robot.motorRight.setPower(0);
        robot.motorLeft.setPower(0);
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
    public void encoderDrive(double leftInches, double rightInches, double fowardInches, double timeoutS, double Speed)
    {
        int newLeftTarget;
        int newRightTarget;
        int newCenterTarget;

        leftInches *= -1;
        rightInches *= -1;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newCenterTarget = robot.motorCenter.getCurrentPosition() + (int)(fowardInches*COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);
            robot.motorCenter.setTargetPosition(newCenterTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(Speed));
            robot.motorRight.setPower(Math.abs(Speed));
            robot.motorCenter.setPower(Math.abs(Speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy() || robot.motorCenter.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget, newCenterTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition(),
                        robot.motorCenter.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
            robot.motorCenter.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}




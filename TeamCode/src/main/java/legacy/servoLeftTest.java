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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import legacy.FoOHardware;

/**
 * Created by coolb on 12/20/2017
 */

@Disabled
@Autonomous(name = "ServoLeftTest", group = "Force of Oros")
public class servoLeftTest extends LinearOpMode
{
    /* Declare OpMode members. */
    FoOHardwareREV robot   = new FoOHardwareREV();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Rev Core Hex Motors
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .4;
    static final double TURN_SPEED = .15;

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

        //sets power when going to zero to brake so that it stops falling
        robot.verticalarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        //put telemetry for xyz here

        //Vuforia Message storage do know what glpyh to put it in
        String glypos = "not visible";

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.right.getCurrentPosition(),
                robot.left.getCurrentPosition());
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

        updateAngles();
        setCardinalDir();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        double x = 0;
        for (int i = 0; i < 11; i++) {
            x = x + .1 * i;
            robot.vaServoLeftT.setPosition(x);
            telemetry.addData("posRT", robot.vaServoLeftT.getPosition());
            telemetry.update();
            x = 0;
            sleep(1000);

        }

        x = 0;
        for (int i = 0; i < 11; i++)
        {
            x = x + .1 * i;
            robot.vaServoLeftB.setPosition(x);
            telemetry.addData("posRT", robot.vaServoLeftB.getPosition());
            telemetry.update();
            x = 0;
            sleep(1000);
        }
    }

    /* (LEGACY)
    public void moveAndCheck(int count)
    {
        for(int i = 0; i < count; i++)
        {
            //turnToNorth();
            //encoderDrive(0, 0, -.5, 2, .2);
            sleep(100);
            robot.touchArm.setPosition(0);
            while(robot.touchRed.getState() && opModeIsActive())
            {
                robot.motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.motorCenter.setPower(-.2);
            }
            robot.motorCenter.setPower(0);
            robot.touchArm.setPosition(1);
            sleep(500);
        }
        //encoderDrive(0, 0, 2, 5, DRIVE_SPEED);
        sleep(100);
        //letting go of glyph
        robot.vaServoLeft.setPosition(.2);
        robot.vaServoRight.setPosition(.8);
        sleep(100);
        encoderDrive(3., 3., 5., DRIVE_SPEED);
    }
    */

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

    public void turnTo(float dir)
    {
        //should work
        updateAngles();
        double currentAngle = angles.firstAngle;
        double desiredAngle = dir;
        if (desiredAngle > 180) desiredAngle -= 360;
        if (desiredAngle < 180) desiredAngle += 360;

        double target = currentAngle - desiredAngle;

        if (target > 180) target -= 360;
        if (target < -180) target += 360;
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
        if (error < -180) error += 360;

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
                //right motors one way...
                robot.right.setPower(powerScaled);

                //left motors the other...
                robot.left.setPower(-powerScaled);
            }
            else if(error < 0)
            {
                //right motors one way...
                robot.right.setPower(-powerScaled);

                //left motors the other...
                robot.left.setPower(powerScaled);
            }
        }
        while ((errorAbs > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

        robot.right.setPower(0);
        robot.left.setPower(0);
    }

    //run this everytime you need to update a direction variable
    public void updateAngles()
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double readAngle(String xyz)
    {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(xyz.equals("x")){
            return angles.thirdAngle;
        }else if(xyz.equals("y")){
            return angles.secondAngle;
        }else if(xyz.equals("z")){
            return angles.firstAngle;
        }else{
            return 0;
        }
    }


    public void setCardinalDir()
    {
        updateAngles();
        NORTH = angles.firstAngle;
        EAST = NORTH + 90;
        if (EAST >= 180)
        {
            EAST -= 360;
        }
        else if (EAST <= -180)
        {
            EAST += 360;
        }
        SOUTH = NORTH + 180;
        if (SOUTH >= 180)
        {
            SOUTH -= 360;
        }
        else if (SOUTH <= -180)
        {
            SOUTH += 360;
        }
        WEST = NORTH - 90;
        if (WEST >= 180)
        {
            WEST -= 360;
        }
        else if (WEST <= 180)
        {
            WEST += 360;
        }
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double leftInches, double rightInches,double sideInches, double timeoutS, double Speed)
    {
        int leftTarget;
        int rightTarget;
        int centerTarget;

        leftInches *= -1;
        rightInches *= -1;
        sideInches *= -1;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftTarget = robot.left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            rightTarget = robot.right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            centerTarget = robot.center.getCurrentPosition() + (int) (sideInches * COUNTS_PER_INCH);
            robot.left.setTargetPosition(leftTarget);
            robot.right.setTargetPosition(rightTarget);
            robot.center.setTargetPosition(centerTarget);

            // Turn On RUN_TO_POSITION
            robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.center.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left.setPower(Math.abs(Speed));
            robot.right.setPower(Math.abs(Speed));
            robot.center.setPower(Math.abs(Speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.right.isBusy() && robot.left.isBusy() && robot.center.isBusy()))
            {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", leftTarget,  rightTarget, centerTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d", robot.left.getCurrentPosition(), robot.right.getCurrentPosition(), robot.center.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.left.setPower(0);
            robot.right.setPower(0);
            robot.center.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}
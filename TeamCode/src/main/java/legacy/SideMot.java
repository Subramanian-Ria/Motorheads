package legacy;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by coolb on 12/20/2017
 */

@Disabled
@Autonomous(name = "SideMOt", group = "Force of Oros")
public class SideMot extends LinearOpMode
{
    /* Declare OpMode members. */
    FoOHardwareREV robot   = new FoOHardwareREV();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // Currently: Andymark Neverest 40
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .4;
    static final double TURN_SPEED = .025;

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
        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.right.getCurrentPosition(),
                robot.left.getCurrentPosition());
        robot.center.getCurrentPosition();
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


        telemetry.addData("Z", readAngle("z"));
        telemetry.addData("y", readAngle("y"));
        telemetry.addData("x", readAngle("x"));
        telemetry.update();

        //robot.left.setPower(.5);
        //robot.right.setPower(.5);
        telemetry.addData("Position right",  robot.right.getCurrentPosition());
        telemetry.addData("Position left",  robot.left.getCurrentPosition());
        telemetry.addData("Position center",  robot.center.getCurrentPosition());
        telemetry.addData("Reached", "pos start");
        telemetry.update();
        robot.left.setPower(.5);
        robot.right.setPower(.5);

        while(getRuntime() < 5)
        {
            telemetry.addData("Position right",  robot.right.getCurrentPosition());
            telemetry.addData("Position left",  robot.left.getCurrentPosition());

            telemetry.update();
        }
        sleep(1000);
        telemetry.addData("Foward data added", "hi" );





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
            telemetry.addData("leftT", leftTarget);
            telemetry.addData("rightT", rightTarget);
            telemetry.addData("centerT", centerTarget);
            telemetry.addData("Reached", "pos target");
            telemetry.update();
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

    @Disabled
    @Autonomous(name = "MecanumAutonBlueDepoMain", group = "Testing")
    public static class MecanumAutonBlueDepoMain extends LinearOpMode
    {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        private ElapsedTime runtime = new ElapsedTime();


        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //turn headings
        public float NORTH;
        public float EAST;
        public float WEST;
        public float SOUTH;

        static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
        static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
        static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
        static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
        static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED = .5;
        static final double TURN_SPEED = .2;

        //Encoder position tracking variables
        double lefttrack;
        double righttrack;

        double lefttarget;
        double righttarget;


        public void runOpMode()
        {
            robot.init(hardwareMap);

            //run using and stop and reset encoders for all relevant motors
            stopAndReset();

            waitForStart();
            encoderElevator(1, -8.4,40);
            gyroinit();
            //BACKS OUT FROM HOOK
            encoderDrive(1,"b",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(4,"r",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(.7,"f",5, DRIVE_SPEED);
            sleep(200);
            //Knocks out center mineral
            encoderDrive(29,"r",10, DRIVE_SPEED);
            sleep(200);
            //turns/moves to deposit marker
            turnDegrees(-43,TURN_SPEED,2.2);

            dropAmerica();
            sleep(500);
            //turnDegrees(30,TURN_SPEED, 5);//TODO: FIND OUT WHY THIS TURNS THE WRONG WAY
            /*sleep(500);
            //drive to crater
            encoderDrive(40,"f", 15,.6);*/
            while(robot.sensordistdepo.getDistance(DistanceUnit.INCH) > 3.5)
            {
                telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(-.2);
                robot.fRMotor.setPower(.2);
                robot.bLMotor.setPower(.2);
                robot.bRMotor.setPower(-.2);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.update();

            //encoderDrive(30,"f", 15,DRIVE_SPEED);
            runtime.reset();
            while(readAngle("x") < 2.5 || runtime.seconds() < 7)
            {
                telemetry.addData("Z", readAngle("z"));
                telemetry.addData("y", readAngle("y"));
                telemetry.addData("x", readAngle("x"));
                telemetry.addData("time", runtime.seconds());
                telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                if(readAngle("z") < 47)
                {

                    telemetry.addData("C1:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //foward
                    robot.fLMotor.setPower(-.5);
                    robot.fRMotor.setPower(-.5);
                    robot.bLMotor.setPower(-.5);
                    robot.bRMotor.setPower(-.5);
                }
                else if(robot.sensordistdepo.getDistance(DistanceUnit.INCH) < 3.5)
                {
                    telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(.2);
                    robot.fRMotor.setPower(-.2);
                    robot.bLMotor.setPower(-.2);
                    robot.bRMotor.setPower(.2);
                }
                else if(robot.sensordistdepo.getDistance(DistanceUnit.INCH) > 6)
                {
                    telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(-.2);
                    robot.fRMotor.setPower(.2);
                    robot.bLMotor.setPower(.2);
                    robot.bRMotor.setPower(-.2);
                }
                else
                {
                    telemetry.addData("C2:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //right?
                    robot.fRMotor.setPower(.2);
                    robot.bRMotor.setPower(.2);
                    robot.fLMotor.setPower(-.2);
                    robot.bLMotor.setPower(-.2);
                }

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);





        }
        public void stopAndReset() {
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.fLMotor.setPower(Math.abs(Speed));
                robot.fRMotor.setPower(Math.abs(Speed));
                robot.bRMotor.setPower(Math.abs(Speed));
                robot.bLMotor.setPower(Math.abs(Speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                    telemetry.update();
                }

                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
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
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runtime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
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
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
        /*
        public void colorSensor() {
            float alpha;
            float red;
            float green;
            float blue;
            float redDifGreen;
            float redDifBlue;
            float blueDifGreen;
            float blueDifRed;

            alpha = robot.sensorCol.alpha();
            red = robot.sensorCol.red();
            green = robot.sensorCol.green();
            blue = robot.sensorCol.blue();

            redDifGreen = red - green;
            redDifBlue = red - blue;
            blueDifGreen = blue - green;
            blueDifRed = blue - red;

            if (redDifBlue > 100) && (redDifGreen > 100) {

            }

            if (blueDifRed > 100) && (blueDifGreen > 100) {

            }
        }
        */
        public void encoderElevator(double speed,double distance, double timeoutS) {
            int newElevatorTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_MOTOR_REV);
                robot.elevator.setTargetPosition(newElevatorTarget);

                // Turn On RUN_TO_POSITION
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.elevator.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.elevator.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.elevator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.elevator.setPower(0);

                // Reset encoders
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //  sleep(250);   // optional pause after each move

            }
        }

        public void dropAmerica()
        {
            robot.armEx.setPower(.3);
            sleep( 750);
            robot.armEx.setPower(0);
            for(int i = 3; i <11; i++)
            {
                robot.bucket.setPosition(.1*i);
                telemetry.addData("pos", .1*i);
                telemetry.update();
                sleep(100);
            }

        }
        public void gyroinit()
        {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
            Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            Gparameters.loggingEnabled = true;
            Gparameters.loggingTag = "IMU";
            Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gparameters);

            updateAngles();
            NORTH = angles.firstAngle;
        }
    }

    @Disabled
    @Autonomous(name = "MecanumAutonElevatorOnly", group = "Testing")
    public static class MecanumAutonElevatorOnly extends LinearOpMode
    {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        private ElapsedTime runtime = new ElapsedTime();


        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //turn headings
        public float NORTH;
        public float EAST;
        public float WEST;
        public float SOUTH;

        static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
        static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
        static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
        static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
        static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED = .5;
        static final double TURN_SPEED = .2;

        //Encoder position tracking variables
        double lefttrack;
        double righttrack;

        double lefttarget;
        double righttarget;


        public void runOpMode()
        {
            robot.init(hardwareMap);

            //run using and stop and reset encoders for all relevant motors
            stopAndReset();

            waitForStart();
            encoderElevator(1, -8.4,40);
            gyroinit();
            //BACKS OUT FROM HOOK
            encoderDrive(1,"b",10, DRIVE_SPEED);
            sleep(200);
            /*encoderDrive(4,"r",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(.7,"f",5, DRIVE_SPEED);
            sleep(200);
            //Knocks out center mineral
            encoderDrive(29,"r",10, DRIVE_SPEED);
            sleep(200);
            //turns/moves to deposit marker
            turnDegrees(-43,TURN_SPEED,2.2);

            dropAmerica();
            sleep(500);
            //turnDegrees(30,TURN_SPEED, 5);//TODO: FIND OUT WHY THIS TURNS THE WRONG WAY
            /*sleep(500);
            //drive to crater
            encoderDrive(40,"f", 15,.6);*/
           /* while(robot.sensordistdepo.getDistance(DistanceUnit.INCH) > 3.5)
            {
                telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(-.2);
                robot.fRMotor.setPower(.2);
                robot.bLMotor.setPower(.2);
                robot.bRMotor.setPower(-.2);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.update();

            //encoderDrive(30,"f", 15,DRIVE_SPEED);
            runtime.reset();
            while(readAngle("x") < 2.5 || runtime.seconds() < 7)
            {
                telemetry.addData("Z", readAngle("z"));
                telemetry.addData("y", readAngle("y"));
                telemetry.addData("x", readAngle("x"));
                telemetry.addData("time", runtime.seconds());
                telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                if(readAngle("z") < 47)
                {

                    telemetry.addData("C1:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //foward
                    robot.fLMotor.setPower(-.5);
                    robot.fRMotor.setPower(-.5);
                    robot.bLMotor.setPower(-.5);
                    robot.bRMotor.setPower(-.5);
                }
                else if(robot.sensordistdepo.getDistance(DistanceUnit.INCH) < 3.5)
                {
                    telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(.2);
                    robot.fRMotor.setPower(-.2);
                    robot.bLMotor.setPower(-.2);
                    robot.bRMotor.setPower(.2);
                }
                else if(robot.sensordistdepo.getDistance(DistanceUnit.INCH) > 6)
                {
                    telemetry.addData("dist:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(-.2);
                    robot.fRMotor.setPower(.2);
                    robot.bLMotor.setPower(.2);
                    robot.bRMotor.setPower(-.2);
                }
                else
                {
                    telemetry.addData("C2:",(robot.sensordistdepo.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //right?
                    robot.fRMotor.setPower(.2);
                    robot.bRMotor.setPower(.2);
                    robot.fLMotor.setPower(-.2);
                    robot.bLMotor.setPower(-.2);
                }

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);*/





        }
        public void stopAndReset() {
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.fLMotor.setPower(Math.abs(Speed));
                robot.fRMotor.setPower(Math.abs(Speed));
                robot.bRMotor.setPower(Math.abs(Speed));
                robot.bLMotor.setPower(Math.abs(Speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                    telemetry.update();
                }

                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
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
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runtime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
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
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
        /*
        public void colorSensor() {
            float alpha;
            float red;
            float green;
            float blue;
            float redDifGreen;
            float redDifBlue;
            float blueDifGreen;
            float blueDifRed;

            alpha = robot.sensorCol.alpha();
            red = robot.sensorCol.red();
            green = robot.sensorCol.green();
            blue = robot.sensorCol.blue();

            redDifGreen = red - green;
            redDifBlue = red - blue;
            blueDifGreen = blue - green;
            blueDifRed = blue - red;

            if (redDifBlue > 100) && (redDifGreen > 100) {

            }

            if (blueDifRed > 100) && (blueDifGreen > 100) {

            }
        }
        */
        public void encoderElevator(double speed,double distance, double timeoutS) {
            int newElevatorTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_MOTOR_REV);
                robot.elevator.setTargetPosition(newElevatorTarget);

                // Turn On RUN_TO_POSITION
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.elevator.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.elevator.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.elevator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.elevator.setPower(0);

                // Reset encoders
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //  sleep(250);   // optional pause after each move

            }
        }

        public void dropAmerica()
        {
            robot.armEx.setPower(.3);
            sleep( 650);
            robot.armEx.setPower(0);
            for(int i = 3; i <11; i++)
            {
                robot.bucket.setPosition(.1*i);
                telemetry.addData("pos", .1*i);
                telemetry.update();
                sleep(100);
            }

        }
        public void gyroinit()
        {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
            Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            Gparameters.loggingEnabled = true;
            Gparameters.loggingTag = "IMU";
            Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gparameters);

            updateAngles();
            NORTH = angles.firstAngle;
        }
    }

    @Disabled
    @Autonomous(name = "MecanumAutonRedCraterBackup", group = "Testing")
    public static class MecanumAutonRedCraterBackup extends LinearOpMode
    {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        private ElapsedTime runtime = new ElapsedTime();


        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //turn headings
        public float NORTH;
        public float EAST;
        public float WEST;
        public float SOUTH;

        static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
        static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
        static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
        static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
        static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED = .6;
        static final double TURN_SPEED = .25;

        //Encoder position tracking variables
        double lefttrack;
        double righttrack;

        double lefttarget;
        double righttarget;


        public void runOpMode()
        {
            robot.init(hardwareMap);

            //run using and stop and reset encoders for all relevant motors
            stopAndReset();

            waitForStart();
            encoderElevator(1, -8.4,40);
            gyroinit();
            //BACKS OUT FROM HOOK
            encoderDrive(1,"b",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(4.5,"r",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(.7,"f",5, DRIVE_SPEED);
            sleep(200);

            //Knocks out center mineral
            encoderDrive(14,"r",10, DRIVE_SPEED);
            sleep(200);

            //go back
            encoderDrive(3,"l",10, DRIVE_SPEED);
            sleep(200);

            //Go to the wall
            encoderDrive(14,"f",10, DRIVE_SPEED);
            sleep(200);

            //turns/moves to deposit marker
            turnDegrees(133,TURN_SPEED,4.5);
            while(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.9)
            {
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.35);
                robot.fRMotor.setPower(-.35);
                robot.bLMotor.setPower(-.35);
                robot.bRMotor.setPower(.35);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.update();

            encoderDrive(18,"b",10, DRIVE_SPEED);
            sleep(200);

            //back to the wall again(avoid hitting silver)
            while(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.2)
            {
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.35);
                robot.fRMotor.setPower(-.35);
                robot.bLMotor.setPower(-.35);
                robot.bRMotor.setPower(.35);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);

            dropAmerica();
            sleep(500);


            while(readAngle("x") < 2.5 || runtime.seconds() < 30)
            {

                telemetry.addData("Z", readAngle("z"));
                telemetry.addData("y", readAngle("y"));
                telemetry.addData("x", readAngle("x"));
                telemetry.addData("time", runtime.seconds());
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();

                if(Math.abs(readAngle("z")) > 130)
                {

                    telemetry.addData("C1:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //foward
                    robot.fLMotor.setPower(-.6);
                    robot.fRMotor.setPower(-.6);
                    robot.bLMotor.setPower(-.6);
                    robot.bRMotor.setPower(-.6);
                }
                else if(robot.sensordist.getDistance(DistanceUnit.INCH) < 3.5)
                {
                    telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(-.35);
                    robot.fRMotor.setPower(.35);
                    robot.bLMotor.setPower(.35);
                    robot.bRMotor.setPower(-.35);
                }
                else
                {
                    telemetry.addData("C2:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //right?
                    robot.fRMotor.setPower(.1);
                    robot.bRMotor.setPower(.1);
                    robot.fLMotor.setPower(-.1);
                    robot.bLMotor.setPower(-.1);
                }

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);





        }
        public void stopAndReset() {
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.fLMotor.setPower(Math.abs(Speed));
                robot.fRMotor.setPower(Math.abs(Speed));
                robot.bRMotor.setPower(Math.abs(Speed));
                robot.bLMotor.setPower(Math.abs(Speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                    telemetry.update();
                }

                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
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
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runtime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
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
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
        /*
        public void colorSensor() {
            float alpha;
            float red;
            float green;
            float blue;
            float redDifGreen;
            float redDifBlue;
            float blueDifGreen;
            float blueDifRed;

            alpha = robot.sensorCol.alpha();
            red = robot.sensorCol.red();
            green = robot.sensorCol.green();
            blue = robot.sensorCol.blue();

            redDifGreen = red - green;
            redDifBlue = red - blue;
            blueDifGreen = blue - green;
            blueDifRed = blue - red;

            if (redDifBlue > 100) && (redDifGreen > 100) {

            }

            if (blueDifRed > 100) && (blueDifGreen > 100) {

            }
        }
        */
        public void encoderElevator(double speed,double distance, double timeoutS) {
            int newElevatorTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_MOTOR_REV);
                robot.elevator.setTargetPosition(newElevatorTarget);

                // Turn On RUN_TO_POSITION
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.elevator.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.elevator.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.elevator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.elevator.setPower(0);

                // Reset encoders
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //  sleep(250);   // optional pause after each move

            }
        }

        public void dropAmerica()
        {
            robot.armEx.setPower(.5);
            sleep(1500);
            robot.armEx.setPower(0);
            robot.bucket.setPosition(1);

        }
        public void gyroinit()
        {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
            Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            Gparameters.loggingEnabled = true;
            Gparameters.loggingTag = "IMU";
            Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gparameters);

            updateAngles();
            NORTH = angles.firstAngle;
        }
    }

    @Disabled
    @Autonomous(name = "MecanumAutonRedCraterMain", group = "Testing")
    public static class MecanumAutonRedCraterMain extends LinearOpMode
    {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        private ElapsedTime runtime = new ElapsedTime();


        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //turn headings
        public float NORTH;
        public float EAST;
        public float WEST;
        public float SOUTH;

        static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
        static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
        static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
        static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
        static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED = .6;
        static final double TURN_SPEED = .25;

        //Encoder position tracking variables
        double lefttrack;
        double righttrack;

        double lefttarget;
        double righttarget;


        public void runOpMode()
        {
            robot.init(hardwareMap);

            //run using and stop and reset encoders for all relevant motors
            stopAndReset();

            waitForStart();
            encoderElevator(1, -8.4,40);
            gyroinit();
            //BACKS OUT FROM HOOK
            encoderDrive(1,"b",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(4.5,"r",10, DRIVE_SPEED);
            sleep(200);
            encoderDrive(.7,"f",5, DRIVE_SPEED);
            sleep(200);

            //Knocks out center mineral
            encoderDrive(14,"r",10, DRIVE_SPEED);
            sleep(200);

            //go back
            encoderDrive(3,"l",10, DRIVE_SPEED);
            sleep(200);

            //Go to the wall
            encoderDrive(14,"f",10, DRIVE_SPEED);
            sleep(200);

            //turns/moves to deposit marker
            turnDegrees(133,TURN_SPEED,4.5);
            while(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.9)
            {
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.35);
                robot.fRMotor.setPower(-.35);
                robot.bLMotor.setPower(-.35);
                robot.bRMotor.setPower(.35);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.update();

            encoderDrive(18,"b",10, DRIVE_SPEED);
            sleep(200);

            //back to the wall again(avoid hitting silver)
            while(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.2)
            {
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.35);
                robot.fRMotor.setPower(-.35);
                robot.bLMotor.setPower(-.35);
                robot.bRMotor.setPower(.35);

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);

            dropAmerica();
            sleep(500);


            while(readAngle("x") < 2.5 || runtime.seconds() < 30)
            {

                telemetry.addData("Z", readAngle("z"));
                telemetry.addData("y", readAngle("y"));
                telemetry.addData("x", readAngle("x"));
                telemetry.addData("time", runtime.seconds());
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();

                if(Math.abs(readAngle("z")) > 130)
                {

                    telemetry.addData("C1:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //foward
                    robot.fLMotor.setPower(-.6);
                    robot.fRMotor.setPower(-.6);
                    robot.bLMotor.setPower(-.6);
                    robot.bRMotor.setPower(-.6);
                }
                else if(robot.sensordist.getDistance(DistanceUnit.INCH) < 3.5)
                {
                    telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    robot.fLMotor.setPower(-.35);
                    robot.fRMotor.setPower(.35);
                    robot.bLMotor.setPower(.35);
                    robot.bRMotor.setPower(-.35);
                }
                else
                {
                    telemetry.addData("C2:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //right?
                    robot.fRMotor.setPower(.1);
                    robot.bRMotor.setPower(.1);
                    robot.fLMotor.setPower(-.1);
                    robot.bLMotor.setPower(-.1);
                }

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);





        }
        public void stopAndReset() {
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.fLMotor.setPower(Math.abs(Speed));
                robot.fRMotor.setPower(Math.abs(Speed));
                robot.bRMotor.setPower(Math.abs(Speed));
                robot.bLMotor.setPower(Math.abs(Speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                    telemetry.update();
                }

                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
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
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runtime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
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
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
        /*
        public void colorSensor() {
            float alpha;
            float red;
            float green;
            float blue;
            float redDifGreen;
            float redDifBlue;
            float blueDifGreen;
            float blueDifRed;

            alpha = robot.sensorCol.alpha();
            red = robot.sensorCol.red();
            green = robot.sensorCol.green();
            blue = robot.sensorCol.blue();

            redDifGreen = red - green;
            redDifBlue = red - blue;
            blueDifGreen = blue - green;
            blueDifRed = blue - red;

            if (redDifBlue > 100) && (redDifGreen > 100) {

            }

            if (blueDifRed > 100) && (blueDifGreen > 100) {

            }
        }
        */
        public void encoderElevator(double speed,double distance, double timeoutS) {
            int newElevatorTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_MOTOR_REV);
                robot.elevator.setTargetPosition(newElevatorTarget);

                // Turn On RUN_TO_POSITION
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.elevator.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.elevator.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.elevator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.elevator.setPower(0);

                // Reset encoders
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //  sleep(250);   // optional pause after each move

            }
        }

        public void dropAmerica()
        {
            robot.armEx.setPower(.3);
            sleep(750);
            robot.armEx.setPower(0);
            for(int i = 3; i <11; i++)
            {
                robot.bucket.setPosition(.1*i);
                telemetry.addData("pos", .1*i);
                telemetry.update();
                sleep(100);
            }

        }
        public void gyroinit()
        {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
            Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            Gparameters.loggingEnabled = true;
            Gparameters.loggingTag = "IMU";
            Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gparameters);

            updateAngles();
            NORTH = angles.firstAngle;
        }
    }

    @Autonomous(name = "FowardTest", group = "Testing")
    public static class FowardTest extends LinearOpMode
    {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        private ElapsedTime runtime = new ElapsedTime();


        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry
        Orientation angles;
        Acceleration gravity;

        //turn headings
        public float NORTH;
        public float EAST;
        public float WEST;
        public float SOUTH;

        static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
        static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
        static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
        static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
        static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double     DRIVE_SPEED = .5;
        static final double TURN_SPEED = .2;

        //Encoder position tracking variables
        double lefttrack;
        double righttrack;

        double lefttarget;
        double righttarget;


        public void runOpMode()
        {
            robot.init(hardwareMap);

            //run using and stop and reset encoders for all relevant motors
            stopAndReset();

            waitForStart();
            gyroinit();

            runtime.reset();
            telemetry.addData("time2:", runtime.seconds());
            telemetry.update();
            //encoderDrive(30,"f", 15,DRIVE_SPEED);
            while(runtime.seconds() < 9)
            {
                telemetry.addData("time", runtime.seconds());
                telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                if(robot.sensordist.getDistance(DistanceUnit.INCH) > 5)
                {

                    telemetry.addData("C1:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //foward
                    robot.fLMotor.setPower(-.5);
                    robot.fRMotor.setPower(-.5);
                    robot.bLMotor.setPower(-.5);
                    robot.bRMotor.setPower(-.5);
                }
                else
                {
                    telemetry.addData("C2:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    //right?
                    robot.fRMotor.setPower(TURN_SPEED);
                    robot.bRMotor.setPower(TURN_SPEED);
                    robot.fLMotor.setPower(-TURN_SPEED);
                    robot.bLMotor.setPower(-TURN_SPEED);
                }

            }
            robot.fLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.bRMotor.setPower(0);
            sleep(100);





        }
        public void stopAndReset() {
            robot.fLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runtime.reset();
                robot.fLMotor.setPower(Math.abs(Speed));
                robot.fRMotor.setPower(Math.abs(Speed));
                robot.bRMotor.setPower(Math.abs(Speed));
                robot.bLMotor.setPower(Math.abs(Speed));


                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                    telemetry.update();
                }

                // Stop all motion;
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //  sleep(250);   // optional pause after each move
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
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runtime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
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
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
        /*
        public void colorSensor() {
            float alpha;
            float red;
            float green;
            float blue;
            float redDifGreen;
            float redDifBlue;
            float blueDifGreen;
            float blueDifRed;

            alpha = robot.sensorCol.alpha();
            red = robot.sensorCol.red();
            green = robot.sensorCol.green();
            blue = robot.sensorCol.blue();

            redDifGreen = red - green;
            redDifBlue = red - blue;
            blueDifGreen = blue - green;
            blueDifRed = blue - red;

            if (redDifBlue > 100) && (redDifGreen > 100) {

            }

            if (blueDifRed > 100) && (blueDifGreen > 100) {

            }
        }
        */
        public void encoderElevator(double speed,double distance, double timeoutS) {
            int newElevatorTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_MOTOR_REV);
                robot.elevator.setTargetPosition(newElevatorTarget);

                // Turn On RUN_TO_POSITION
                robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.elevator.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.elevator.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                    telemetry.addData("Path2",  "Running at %7d",
                            robot.elevator.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                robot.elevator.setPower(0);

                // Reset encoders
                robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //  sleep(250);   // optional pause after each move

            }
        }

        public void dropAmerica()
        {
            robot.intake.setPower(-.5);
            sleep(1500);
            robot.intake.setPower(0);
        }
        public void gyroinit()
        {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
            Gparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            Gparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            Gparameters.loggingEnabled = true;
            Gparameters.loggingTag = "IMU";
            Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(Gparameters);

            updateAngles();
            NORTH = angles.firstAngle;
        }
    }

    @Autonomous(name="Drive Avoid PID", group="Testing")
    //@Disabled
    public static class PIDMecanumTest extends LinearOpMode {
        servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
        ElapsedTime runTime = new ElapsedTime();
        //DcMotor leftMotor;
        //DcMotor rightMotor;
        DigitalChannel touch;
        BNO055IMU imu;
        Orientation lastAngles = new Orientation();
        double globalAngle, power = .30, correction;
        //boolean aButton, bButton, touched;
        PIDController pidRotate, pidDrive;

        // called when init button is  pressed.
        @Override
        public void runOpMode() throws InterruptedException {


            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            imu.initialize(parameters);

            // Set PID proportional value to start reducing power at about 50 degrees of rotation.
            pidRotate = new PIDController(.005, 0, 0);

            // Set PID proportional value to produce non-zero correction value when robot veers off
            // straight line. P value controls how sensitive the correction is.
            pidDrive = new PIDController(.05, 0, 0);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            telemetry.addData("Mode", "waiting for start");
            telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
            telemetry.update();

            // wait for start button.

            waitForStart();

            telemetry.addData("Mode", "running");
            telemetry.update();

            sleep(1000);

            // Set up parameters for driving in a straight line.
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();

            // drive until end of period.

            while (opModeIsActive()) {
                // Use PID with imu input to drive in a straight line.
                correction = pidDrive.performPID(getAngle());

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();

                // set power levels.
                robot.fLMotor.setPower(-power + correction);
                robot.bLMotor.setPower(-power + correction);
                robot.fRMotor.setPower(-power);
                robot.bRMotor.setPower(-power);

                // We record the sensor values because we will test them in more than
                // one place with time passing between those places. See the lesson on
                // Timing Considerations to know why.

                if (gamepad1.a || gamepad1.b) {
                    // backup.
                    robot.fLMotor.setPower(power);
                    robot.bLMotor.setPower(power);
                    robot.fRMotor.setPower(power);
                    robot.bRMotor.setPower(power);

                    sleep(500);

                    // stop.
                    robot.fLMotor.setPower(0);
                    robot.bLMotor.setPower(0);
                    robot.fRMotor.setPower(0);
                    robot.bRMotor.setPower(0);

                    // turn 90 degrees right.
                    if (gamepad1.a) rotate(-90, power);

                    // turn 90 degrees left.
                    if (gamepad1.b) rotate(90, power);
                }
            }

            // turn the motors off.
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
        }

        /**
         * Resets the cumulative angle tracking to zero.
         */
        private void resetAngle() {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }

        /**
         * Get current cumulative angle rotation from last reset.
         *
         * @return Angle in degrees. + = left, - = right from zero point.
         */
        private double getAngle() {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;

            return globalAngle;
        }

        /**
         * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
         *
         * @param degrees Degrees to turn, + is left - is right
         */
        private void rotate(int degrees, double power) {
            // restart imu angle tracking.
            resetAngle();

            // start pid controller. PID controller will monitor the turn angle with respect to the
            // target angle and reduce power as we approach the target angle with a minimum of 20%.
            // This is to prevent the robots momentum from overshooting the turn after we turn off the
            // power. The PID controller reports onTarget() = true when the difference between turn
            // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
            // The minimum power is determined by testing and must enough to prevent motor stall and
            // complete the turn. Note: if the gap between the starting power and the stall (minimum)
            // power is small, overshoot may still occur. Overshoot is dependant on the motor and
            // gearing configuration, starting power, weight of the robot and the on target tolerance.

            pidRotate.reset();
            pidRotate.setSetpoint(degrees);
            pidRotate.setInputRange(0, 90);
            pidRotate.setOutputRange(.20, power);
            pidRotate.setTolerance(2);
            pidRotate.enable();

            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            // rotate until turn is completed.

            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {
                    robot.fLMotor.setPower(-power);
                    robot.bLMotor.setPower(-power);
                    robot.fRMotor.setPower(power);
                    robot.bRMotor.setPower(power);
                    sleep(100);
                }

                do {
                    power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                    robot.fLMotor.setPower(power);
                    robot.bLMotor.setPower(power);
                    robot.fRMotor.setPower(-power);
                    robot.bRMotor.setPower(-power);
                } while (opModeIsActive() && !pidRotate.onTarget());
            } else    // left turn.
                do {
                    power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                    robot.fLMotor.setPower(power);
                    robot.bLMotor.setPower(power);
                    robot.fRMotor.setPower(-power);
                    robot.bRMotor.setPower(-power);
                } while (opModeIsActive() && !pidRotate.onTarget());

            // turn the motors off.
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // wait for rotation to stop.
            sleep(500);

            // reset angle tracking on new heading.
            resetAngle();
        }

        // PID controller courtesy of Peter Tischler, with modifications.

        public class PIDController {
            private double m_P;                                 // factor for "proportional" control
            private double m_I;                                 // factor for "integral" control
            private double m_D;                                 // factor for "derivative" control
            private double m_input;                 // sensor input for pid controller
            private double m_maximumOutput = 1.0;   // |maximum output|
            private double m_minimumOutput = -1.0;  // |minimum output|
            private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
            private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
            private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
            private boolean m_enabled = false;              //is the pid controller enabled
            private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
            private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
            private double m_tolerance = 0.05;          //the percentage error that is considered on target
            private double m_setpoint = 0.0;
            private double m_error = 0.0;
            private double m_result = 0.0;

            /**
             * Allocate a PID object with the given constants for P, I, D
             *
             * @param Kp the proportional coefficient
             * @param Ki the integral coefficient
             * @param Kd the derivative coefficient
             */
            public PIDController(double Kp, double Ki, double Kd) {
                m_P = Kp;
                m_I = Ki;
                m_D = Kd;
            }

            /**
             * Read the input, calculate the output accordingly, and write to the output.
             * This should only be called by the PIDTask
             * and is created during initialization.
             */
            private void calculate() {
                int sign = 1;

                // If enabled then proceed into controller calculations
                if (m_enabled) {
                    // Calculate the error signal
                    m_error = m_setpoint - m_input;

                    // If continuous is set to true allow wrap around
                    if (m_continuous) {
                        if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
                            if (m_error > 0)
                                m_error = m_error - m_maximumInput + m_minimumInput;
                            else
                                m_error = m_error + m_maximumInput - m_minimumInput;
                        }
                    }

                    // Integrate the errors as long as the upcoming integrator does
                    // not exceed the minimum and maximum output thresholds.

                    if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                            (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                        m_totalError += m_error;

                    // Perform the primary PID calculation
                    m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

                    // Set the current error to the previous error for the next cycle.
                    m_prevError = m_error;

                    if (m_result < 0) sign = -1;    // Record sign of result.

                    // Make sure the final result is within bounds. If we constrain the result, we make
                    // sure the sign of the constrained result matches the original result sign.
                    if (Math.abs(m_result) > m_maximumOutput)
                        m_result = m_maximumOutput * sign;
                    else if (Math.abs(m_result) < m_minimumOutput)
                        m_result = m_minimumOutput * sign;
                }
            }

            /**
             * Set the PID Controller gain parameters.
             * Set the proportional, integral, and differential coefficients.
             *
             * @param p Proportional coefficient
             * @param i Integral coefficient
             * @param d Differential coefficient
             */
            public void setPID(double p, double i, double d) {
                m_P = p;
                m_I = i;
                m_D = d;
            }

            /**
             * Get the Proportional coefficient
             *
             * @return proportional coefficient
             */
            public double getP() {
                return m_P;
            }

            /**
             * Get the Integral coefficient
             *
             * @return integral coefficient
             */
            public double getI() {
                return m_I;
            }

            /**
             * Get the Differential coefficient
             *
             * @return differential coefficient
             */
            public double getD() {
                return m_D;
            }

            /**
             * Return the current PID result for the last input set with setInput().
             * This is always centered on zero and constrained the the max and min outs
             *
             * @return the latest calculated output
             */
            public double performPID() {
                calculate();
                return m_result;
            }

            /**
             * Return the current PID result for the specified input.
             *
             * @param input The input value to be used to calculate the PID result.
             *              This is always centered on zero and constrained the the max and min outs
             * @return the latest calculated output
             */
            public double performPID(double input) {
                setInput(input);
                return performPID();
            }

            /**
             * Set the PID controller to consider the input to be continuous,
             * Rather then using the max and min in as constraints, it considers them to
             * be the same point and automatically calculates the shortest route to
             * the setpoint.
             *
             * @param continuous Set to true turns on continuous, false turns off continuous
             */
            public void setContinuous(boolean continuous) {
                m_continuous = continuous;
            }

            /**
             * Set the PID controller to consider the input to be continuous,
             * Rather then using the max and min in as constraints, it considers them to
             * be the same point and automatically calculates the shortest route to
             * the setpoint.
             */
            public void setContinuous() {
                this.setContinuous(true);
            }

            /**
             * Sets the maximum and minimum values expected from the input.
             *
             * @param minimumInput the minimum value expected from the input, always positive
             * @param maximumInput the maximum value expected from the output, always positive
             */
            public void setInputRange(double minimumInput, double maximumInput) {
                m_minimumInput = Math.abs(minimumInput);
                m_maximumInput = Math.abs(maximumInput);
                setSetpoint(m_setpoint);
            }

            /**
             * Sets the minimum and maximum values to write.
             *
             * @param minimumOutput the minimum value to write to the output, always positive
             * @param maximumOutput the maximum value to write to the output, always positive
             */
            public void setOutputRange(double minimumOutput, double maximumOutput) {
                m_minimumOutput = Math.abs(minimumOutput);
                m_maximumOutput = Math.abs(maximumOutput);
            }

            /**
             * Set the setpoint for the PIDController
             *
             * @param setpoint the desired setpoint
             */
            public void setSetpoint(double setpoint) {
                int sign = 1;

                if (m_maximumInput > m_minimumInput) {
                    if (setpoint < 0) sign = -1;

                    if (Math.abs(setpoint) > m_maximumInput)
                        m_setpoint = m_maximumInput * sign;
                    else if (Math.abs(setpoint) < m_minimumInput)
                        m_setpoint = m_minimumInput * sign;
                    else
                        m_setpoint = setpoint;
                } else
                    m_setpoint = setpoint;
            }

            /**
             * Returns the current setpoint of the PIDController
             *
             * @return the current setpoint
             */
            public double getSetpoint() {
                return m_setpoint;
            }

            /**
             * Retruns the current difference of the input from the setpoint
             *
             * @return the current error
             */
            public synchronized double getError() {
                return m_error;
            }

            /**
             * Set the percentage error which is considered tolerable for use with
             * OnTarget. (Input of 15.0 = 15 percent)
             *
             * @param percent error which is tolerable
             */
            public void setTolerance(double percent) {
                m_tolerance = percent;
            }

            /**
             * Return true if the error is within the percentage of the total input range,
             * determined by setTolerance. This assumes that the maximum and minimum input
             * were set using setInputRange.
             *
             * @return true if the error is less than the tolerance
             */
            public boolean onTarget() {
                return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
            }

            /**
             * Begin running the PIDController
             */
            public void enable() {
                m_enabled = true;
            }

            /**
             * Stop running the PIDController.
             */
            public void disable() {
                m_enabled = false;
            }

            /**
             * Reset the previous error,, the integral term, and disable the controller.
             */
            public void reset() {
                disable();
                m_prevError = 0;
                m_totalError = 0;
                m_result = 0;
            }

            /**
             * Set the input value to be used by the next call to performPID().
             *
             * @param input Input value to the PID calculation.
             */
            public void setInput(double input) {
                int sign = 1;

                if (m_maximumInput > m_minimumInput) {
                    if (input < 0) sign = -1;

                    if (Math.abs(input) > m_maximumInput)
                        m_input = m_maximumInput * sign;
                    else if (Math.abs(input) < m_minimumInput)
                        m_input = m_minimumInput * sign;
                    else
                        m_input = input;
                } else
                    m_input = input;
            }
        }
    }
}
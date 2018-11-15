package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




@Autonomous(name = "AutonTest", group = "Testing")
public class MecanumAuton extends LinearOpMode
{
    MecanumHardware robot = new MecanumHardware();
    private ElapsedTime runtime = new ElapsedTime();


    // The IMU sensor object
    BNO055IMU imu;

    // Detector object
    private GoldAlignDetector detector;

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
    static final double     DRIVE_SPEED = .4;
    static final double TURN_SPEED = .11;

    //Encoder position tracking variables
    double lefttrack;
    double righttrack;

    double lefttarget;
    double righttarget;


    public void runOpMode()
    {
        robot.init(hardwareMap);
        boolean mincap = false;


        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();

        //first lower
        //encoderElevator(DRIVE_SPEED, 29.50,30);

        //init gyro callibration
        imuinit();

        //Drive sideways to unhook

        //Do sampling detection

        //1st init detector
        detectinit();

        //mtd 1 test sampling detector to see if it works
        //mtd 2 do a sweep, start right and go left until gold align is true
        //mtd 3 Combine both, use sample detector 1st and use that as initial guess
        //TODO discuss using a color sensor on the botttom of the robot rather hardcode for zone

        //check center first
        if(detector.getAligned())
        {
            //drive to hit the mineral
            encoderDrive(20,20,20,20,5,DRIVE_SPEED);
            mincap = true;

            //drive forward and drop the boi
            encoderDrive(10,10,10,10,5,DRIVE_SPEED);
            dropAmerica();
        }

        //if center dosen't work go right
        else if(mincap == false)
        {
            turnDegrees(30, TURN_SPEED, 2); //TODO test all values
            if(detector.getAligned() == true)
            {
                encoderDrive(25, 25, 25, 25, 5, DRIVE_SPEED);
                mincap = true;
            }

            turnDegrees(270, TURN_SPEED, 2); //
            //turn to wall(assuming EAST rn) drive forward drop the boi
            encoderDrive(10,10,10,10,5,DRIVE_SPEED);
            dropAmerica();



        }

        //if right dosen't work go left
        else if(mincap == false)
        {
            turnDegrees(-60, TURN_SPEED, 2); //TODO test all values
            if(detector.getAligned() == true)
            {
                encoderDrive(25, 25, 25, 25, 5, DRIVE_SPEED);
                mincap = true;
            }

            //turn to wall(assuming NORTH rn) drive forward drop the boi
            turnDegrees(90, TURN_SPEED, 2); //
            //turn to wall(assuming EAST rn) drive forward drop the boi
            encoderDrive(10,10,10,10,5,DRIVE_SPEED);
            dropAmerica();
        }


        //Next drop the sample into the zone

        //Park(Maybe): might not be work since we don't know if we'll get out, lets figure out where we want to end later


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

    public void encoderDrive(double FLInch, double FRInch, double BLInch, double BRInch, double timeoutS, double Speed)
    {

        int TargetFL;
        int TargetFR;
        int TargetBL;
        int TargetBR;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            TargetFL = robot.fLMotor.getCurrentPosition() + (int)( FLInch* COUNTS_PER_INCH);
            TargetFR = robot.fRMotor.getCurrentPosition() + (int)( FRInch* COUNTS_PER_INCH);
            TargetBL = robot.fRMotor.getCurrentPosition() + (int)( BLInch* COUNTS_PER_INCH);
            TargetBR = robot.fRMotor.getCurrentPosition() + (int)( BRInch* COUNTS_PER_INCH);

            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bRMotor.setTargetPosition(TargetFR);
            robot.bLMotor.setTargetPosition(TargetFR);


            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fLMotor.setPower(Speed);
            robot.fRMotor.setPower(Speed);
            robot.bRMotor.setPower(Speed);
            robot.bLMotor.setPower(Speed);


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
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetBL,  TargetBR, TargetFL, TargetFR);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
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
                powerScaled /= 1.5;
            }

            telemetry.addData("error", error);
            telemetry.update();
            if(error > 0)
            {
                //right motors one way...
                robot.fRMotor.setPower(powerScaled);
                robot.bRMotor.setPower(powerScaled);

                //left motors the other...
                robot.fLMotor.setPower(-powerScaled);
                robot.bLMotor.setPower(-powerScaled);
            }
            else if(error < 0)
            {
                //right motors one way...
                robot.fRMotor.setPower(powerScaled);
                robot.bRMotor.setPower(powerScaled);


                //left motors the other...
                robot.fLMotor.setPower(-powerScaled);
                robot.bLMotor.setPower(-powerScaled);
            }
        }
        while ((errorAbs > 1.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

        robot.fLMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bRMotor.setPower(0);
    }
    public void colorSensor() {
        float alpha;
        float red;
        float green;
        float blue;
        float redDifGreen;
        float redDifBlue;
        float blueDifGreen;
        float blueDifRed;

       // alpha = robot.sensorCol.alpha();
        //red = robot.sensorCol.red();
       // green = robot.sensorCol.green();
        //blue = robot.sensorCol.blue();

        //redDifGreen = red - green;
        //redDifBlue = red - blue;
        //blueDifGreen = blue - green;
        //blueDifRed = blue - red;
    /*
        if (redDifBlue > 100) && (redDifGreen > 100) {

        }

        if (blueDifRed > 100) && (blueDifGreen > 100) {

        }
        */
    }

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

    public void detectinit()
    {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

    }
    public void imuinit()
    {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void dropAmerica()
    {
        robot.intake.setPower(1);
        sleep(1500);
        robot.intake.setPower(0);
    }
}

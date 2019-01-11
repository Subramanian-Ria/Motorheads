package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;




@Autonomous(name = "MecanumAutonBlueDepoBack", group = "Testing")
public class MecanumAutonBlueDepoBack extends LinearOpMode
{
    MecanumHardware robot = new MecanumHardware();
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
        encoderElevator(1, -7.9,40);
        gyroinit();
        //BACKS OUT FROM HOOK
        encoderDrive(1,"b",10, DRIVE_SPEED);
        sleep(200);
        encoderDrive(4.5,"r",10, DRIVE_SPEED);
        sleep(200);
        encoderDrive(.7,"f",5, DRIVE_SPEED);
        sleep(200);
        //Knocks out center mineral
        encoderDrive(29,"r",10, DRIVE_SPEED);
        sleep(200);
        //turns/moves to deposit marker
        turnDegrees(-133,TURN_SPEED,5.5);

        //dropAmerica();
        //turnDegrees(30,TURN_SPEED, 5);//TODO: FIND OUT WHY THIS TURNS THE WRONG WAY
        /*sleep(500);
        //drive to crater
        encoderDrive(40,"f", 15,.6);*/
        while(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.6)
        {
            telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            robot.fLMotor.setPower(.2);
            robot.fRMotor.setPower(-.2);
            robot.bLMotor.setPower(-.2);
            robot.bRMotor.setPower(.2);

        }
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
        sleep(100);
        runtime.reset();
        //encoderDrive(30,"f", 15,DRIVE_SPEED);
        while(runtime.seconds() < 7)
        {
            telemetry.addData("time", runtime.seconds());
            telemetry.addData("dist:",(robot.sensordist.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            if(robot.sensordist.getDistance(DistanceUnit.INCH) > 4.5)
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
                robot.fRMotor.setPower(.07);
                robot.bRMotor.setPower(.07);
                robot.fLMotor.setPower(-.07);
                robot.bLMotor.setPower(-.07);
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
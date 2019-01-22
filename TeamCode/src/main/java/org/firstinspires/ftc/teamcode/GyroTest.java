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




@Autonomous(name = "GYROTEST", group = "Testing")
public class GyroTest extends LinearOpMode
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
        stopAndReset();
        waitForStart();
        turnDegrees(90,TURN_SPEED,4.5);
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
    public void turnDegrees(double relTarget, double power, double timeoutS)
    {
        //Write code to correct to a target position (NOT FINISHED)
        runtime.reset();
        updateAngles(); //variable for gyro correction around z axis//switches clockwise and counterclockwise directions
        double absTarget = angles.firstAngle + relTarget;
        double error;
        //wrapping error to have it remain in the field
        //if (error > 180)  error -= 360;
        //if (error <= -180) error += 360;

        double powerScaled = power;
        do
        {
            updateAngles();
            error = angles.firstAngle + absTarget;

            if (error <= 5)
            {
                powerScaled /= 2;
            }
            telemetry.addData("error", error);
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
        while ((Math.abs(error) > 0.5) && (runtime.seconds() < timeoutS) && opModeIsActive());

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

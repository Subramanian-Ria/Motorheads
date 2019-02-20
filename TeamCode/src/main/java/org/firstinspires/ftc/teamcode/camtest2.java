package org.firstinspires.ftc.teamcode;


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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;

import legacy.MecanumHardware2;


@Autonomous(name = "camtest2", group = "MecanumBot2")
public class camtest2 extends MecanumDriving
{
    MecanumHardware3 robot = new MecanumHardware3();
    private ElapsedTime runtime = new ElapsedTime();


    // The IMU sensor object
    BNO055IMU imu;

    //tensorflow object
    TensorFlow tf;

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
    static final double TURN_SPEED = .3;

    //Encoder position tracking variables
    double lefttrack;
    double righttrack;

    double lefttarget;
    double righttarget;


    public void runOpMode()
    {
        robot.init(hardwareMap);
        tf = new TensorFlow(hardwareMap, Device.Webcam,telemetry);
        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();
        tf.start();
        sleep(500);

        MineralLocation goldMineralLocation;
        goldMineralLocation = tf.getMineralLocation(RobotOrientation.Left);
        sleep(1000);
        goldMineralLocation = tf.getMineralLocation(RobotOrientation.Left);

        while(runtime.seconds() < 20 || opModeIsActive())
        {

            goldMineralLocation = tf.getMineralLocation(RobotOrientation.Left);
            if (goldMineralLocation == MineralLocation.Left)
            {


                telemetry.addData("Left", (robot.sensorDist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
            }
            else if (goldMineralLocation == MineralLocation.Center)
            {
                telemetry.addData("Center", (robot.sensorDist.getDistance(DistanceUnit.INCH)));
                telemetry.update();
            }
            else
                {
            telemetry.addData("Right", (robot.sensorDist.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            }

        }
        tf.shutdown();
    }

}

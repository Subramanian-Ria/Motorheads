package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;


@Autonomous(name = "MecanumAuton3CraterTestLeft", group = "MecanumBot3")
public class MecanumAuton3CraterTestLeft extends MecanumDriving
{

    public void runOpMode()
    {
        robot.init(hardwareMap);

        tf = new TensorFlow(hardwareMap, Device.Webcam,telemetry);
        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();

        tf.start();

        sleep(250);

        MineralLocation goldMineralLocation;

        encoderElevator(1, -9.542, 40);
        goldMineralLocation = tf.getMineralLocation(RobotOrientation.Left);
        sleep(1000);
        goldMineralLocation = tf.getMineralLocation(RobotOrientation.Left);

        tf.shutdown();

        //BACKS OUT FROM HOOK
        gyroinit();

        encoderDrive(3, "f", 10, DRIVE_SPEED);
        sleep(200);

        encoderDrive(4,"l",10, DRIVE_SPEED);
        sleep(200);
        encoderDrive(2,"b",5, DRIVE_SPEED);
        sleep(200);

        telemetry.addData("Location Left", 0);
        telemetry.update();
        turnDegrees(-35, TURN_SPEED, 2);
        //drive towards mineral
        encoderDrive(14.5, "l", 10, DRIVE_SPEED);
        encoderDrive(4, "r", 10, DRIVE_SPEED);
        sleep(100);
        //turnDegrees(4.5, TURN_SPEED, 10);
        turnDegrees(35, TURN_SPEED, 2);

    }




}

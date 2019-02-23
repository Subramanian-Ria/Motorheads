package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;




@Autonomous(name = "camtest3", group = "MecanumBot3")
public class camtest3 extends MecanumDriving
{


    public void runOpMode() {
        robot.init(hardwareMap);

        tf = new TensorFlow(hardwareMap, Device.Webcam, telemetry);
        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();

        tf.start();

        sleep(250);

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






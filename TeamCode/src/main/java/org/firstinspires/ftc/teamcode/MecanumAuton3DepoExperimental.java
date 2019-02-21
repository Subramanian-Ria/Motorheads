package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;




@Autonomous(name = "MecanumAuton3DepoExp", group = "MecanumBot")
public class MecanumAuton3DepoExperimental extends MecanumDriving
{


    public void runOpMode()
    {
        robot.init(hardwareMap);

        tf = new TensorFlow(hardwareMap, Device.Webcam,telemetry);
        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();
        gyroinit();


        robot.fLMotor.setPower(-.9);
        robot.fRMotor.setPower(-.9);
        robot.bLMotor.setPower(-.9);
        robot.bRMotor.setPower(-.9);
        robot.armEx.setPower(-.75);
        sleep(1100);



        tf.shutdown();






    }




}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;




@Autonomous(name = "MecanumAuton3Crater", group = "MecanumBot3")
public class MecanumAuton3Crater extends MecanumDriving
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

        if(goldMineralLocation == MineralLocation.Left)
        {
            telemetry.addData("Location Left", 0);
            telemetry.update();
            turnDegrees(-33, TURN_SPEED, 2);
            //drive towards mineral
            encoderDrive(14.5, "l", 10, DRIVE_SPEED);
            encoderDrive(2, "b", 10, DRIVE_SPEED);
            encoderDrive(4.3, "r", 10, DRIVE_SPEED);
            sleep(100);
            //turnDegrees(4.5, TURN_SPEED, 10);
            turnDegrees(0, TURN_SPEED, 2);
            encoderDrive(10, "b", 10, DRIVE_SPEED);

        }

        else if(goldMineralLocation == MineralLocation.Center)
        {
            telemetry.addData("Location Center", 0);
            telemetry.update();
            //Knocks out center mineral
            encoderDrive(12, "l", 10, DRIVE_SPEED);
            encoderDrive(3.5, "r", 10, DRIVE_SPEED);
            encoderDrive(14, "b", 10, DRIVE_SPEED);
        }

        else
        {
            telemetry.addData("Location Right", 0);
            telemetry.update();
            turnDegrees(30, TURN_SPEED, 2);
            //drive towards mineral
            encoderDrive(12.5, "l", 10, DRIVE_SPEED);
            encoderDrive(4.3, "r", 10, DRIVE_SPEED);
            sleep(100);
            //turnDegrees(4.5, TURN_SPEED, 10);
            turnDegrees(0, TURN_SPEED, 2);
            encoderDrive(17, "b", 10, DRIVE_SPEED);
        }
        turnDegrees(-43, TURN_SPEED, 2);

        while(robot.sensorDistDepo.getDistance(DistanceUnit.INCH) > 2.2)
        {
            telemetry.addData("dist:",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            robot.fLMotor.setPower(-.4);
            robot.fRMotor.setPower(.4);
            robot.bLMotor.setPower(.4);
            robot.bRMotor.setPower(-.4);

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

        //move arm to clear wires and deposit marker
        robot.armFlip.setPower(-.65);
        sleep(550);
        robot.armFlip.setPower(0);
        //
        robot.fLMotor.setPower(-.9);
        robot.fRMotor.setPower(-.9);
        robot.bLMotor.setPower(-.9);
        robot.bRMotor.setPower(-.9);
        robot.armEx.setPower(-.85);
        sleep(1300);
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
        robot.armEx.setPower(0);
        robot.bucket.setPower(-.25);
        sleep(200);
        robot.bucket.setPower(0);
        robot.armEx.setPower(1);
        sleep(500);
        robot.armEx.setPower(0);

        telemetry.addData("runtime 1", runtime.seconds());
        telemetry.update();
        runtime.reset();
        while(readAngle("y") > -2.5 && opModeIsActive())
        {
            telemetry.addData("y; ", readAngle("y"));
            telemetry.update();
            robot.fLMotor.setPower(.9);
            robot.fRMotor.setPower(.9);
            robot.bLMotor.setPower(.9);
            robot.bRMotor.setPower(.9);
        }
        /*
        while((readAngle("y") > -1.5 || runtime.seconds() < 5) && opModeIsActive())
        {
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.addData("time", runtime.seconds());
            telemetry.addData("dist:",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            if(readAngle("z") > 15)
            {
                telemetry.addData("C1",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fRMotor.setPower(-.25);
                robot.bRMotor.setPower(-.25);
                robot.fLMotor.setPower(.25);
                robot.bLMotor.setPower(.25);
            }
            else if(readAngle("z") < -15)
            {
                telemetry.addData("C1.2",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fRMotor.setPower(.25);
                robot.bRMotor.setPower(.25);
                robot.fLMotor.setPower(-.25);
                robot.bLMotor.setPower(-.25);
            }
            else if(robot.sensorDist.getDistance(DistanceUnit.INCH) < 2.2)
            {
                telemetry.addData("C2",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.25);
                robot.fRMotor.setPower(-.25);
                robot.bLMotor.setPower(-.25);
                robot.bRMotor.setPower(.25);
            }
            else if(robot.sensorDistDepo.getDistance(DistanceUnit.INCH) > 6.5)
            {
                telemetry.addData("C3:",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(-.25);
                robot.fRMotor.setPower(.25);
                robot.bLMotor.setPower(.25);
                robot.bRMotor.setPower(-.25);
            }
            else
            {
                telemetry.addData("C4:",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                //foward
                robot.fLMotor.setPower(.8);
                robot.fRMotor.setPower(.8);
                robot.bLMotor.setPower(.8);
                robot.bRMotor.setPower(.8);
            }

        }
        */
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
        sleep(100);

    }




}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TensorFlow.Device;
import org.firstinspires.ftc.teamcode.TensorFlow.RobotOrientation;
import org.firstinspires.ftc.teamcode.TensorFlow.MineralLocation;




@Autonomous(name = "MecanumAuton3Depo", group = "MecanumBot")
public class MecanumAuton3Depo extends MecanumDriving
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
        encoderElevator(1, -9.542, 40);
        //BACKS OUT FROM HOOK
        tf.shutdown();
        gyroinit();
        encoderDrive(2, "f", 10, DRIVE_SPEED);
        sleep(200);


        encoderDrive(4, "l", 10, DRIVE_SPEED);
        sleep(200);
        encoderDrive(2, "b", 5, DRIVE_SPEED);
        sleep(200);

        if (goldMineralLocation == MineralLocation.Left)
        {
            turnDegrees(-25, TURN_SPEED, 2);
            encoderDrive(17.5, "l", 10, DRIVE_SPEED);
            turnDegrees(-43, TURN_SPEED, 2.5);
            encoderDrive(10, "b", 10, DRIVE_SPEED);

        }
        else if (goldMineralLocation == MineralLocation.Center)
        {
            //Knocks out center mineral
            encoderDrive(24, "l", 10, DRIVE_SPEED);
            sleep(200);
            //turns/moves to deposit marker
            turnDegrees(-43, TURN_SPEED, 3.5);
        }
        else
        {
            turnDegrees(30, TURN_SPEED, 2);
            encoderDrive(17.5, "l", 10, DRIVE_SPEED);
            turnDegrees(-43, TURN_SPEED, 2.5);
            encoderDrive(5, "f", 10, DRIVE_SPEED);
            encoderDrive(15, "l", 10, DRIVE_SPEED);
        }
        //sleep(500);
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

        dropAmerica();

        //encoderDrive(30,"f", 15,DRIVE_SPEED);
        telemetry.addData("runtime 1", runtime.seconds());
        telemetry.update();
        runtime.reset();
        while((readAngle("y") < 1.5 || runtime.seconds() < 5) && opModeIsActive())
        {
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));
            telemetry.addData("time", runtime.seconds());
            telemetry.addData("dist:",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            if(readAngle("z") > 52)
            {
                telemetry.addData("C1",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fRMotor.setPower(-.25);
                robot.bRMotor.setPower(-.25);
                robot.fLMotor.setPower(.25);
                robot.bLMotor.setPower(.25);
            }
            else if(readAngle("z") < 32)
            {
                telemetry.addData("C1.2",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fRMotor.setPower(.25);
                robot.bRMotor.setPower(.25);
                robot.fLMotor.setPower(-.25);
                robot.bLMotor.setPower(-.25);
            }
            else if(robot.sensorDistDepo.getDistance(DistanceUnit.INCH) < 2.2)
            {
                telemetry.addData("C2",(robot.sensorDistDepo.getDistance(DistanceUnit.INCH)));
                telemetry.update();
                robot.fLMotor.setPower(.25);
                robot.fRMotor.setPower(-.25);
                robot.bLMotor.setPower(-.25);
                robot.bRMotor.setPower(.25);
            }
            else if(robot.sensorDistDepo.getDistance(DistanceUnit.INCH) > 6)
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
                robot.fLMotor.setPower(-.8);
                robot.fRMotor.setPower(-.8);
                robot.bLMotor.setPower(-.8);
                robot.bRMotor.setPower(-.8);
            }

        }
        robot.fLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.bRMotor.setPower(0);
        sleep(100);

        tf.shutdown();






    }




}

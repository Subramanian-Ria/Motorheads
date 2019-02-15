package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@Autonomous(name = "FullSensorTest", group = "MecanumBot2")
public class FullSensorTest extends MecanumDriving
{

    public void runOpMode()
    {
        robot.init(hardwareMap);

        //run using and stop and reset encoders for all relevant motors
        stopAndReset();

        waitForStart();

        gyroinit();

        while(1 == 1)
        {
            telemetry.addData("Z", readAngle("z"));
            telemetry.addData("y", readAngle("y"));
            telemetry.addData("x", readAngle("x"));

            telemetry.addData("Distance: ", robot.sensorDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance: ", robot.sensorDistDepo.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance: ", robot.sensorDistElevator.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }



    }

}

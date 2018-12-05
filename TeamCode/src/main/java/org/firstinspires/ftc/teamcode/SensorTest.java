package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

//import legacy.FoOHardware;

@TeleOp(name="SensorTest", group="Testing")
//@Disabled
public class SensorTest extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runTime = new ElapsedTime();


    float rightPower = 0; //motor power for future use
    float leftPower = 0;

    float pLim = .8f; //power multiplier that acts as a limit
    float tHold = .1f;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Alpha: ", robot.sensorCol1.alpha());
        telemetry.addData("Red: ", robot.sensorCol1.red());
        telemetry.addData("Green: ", robot.sensorCol1.green());
        telemetry.addData("Blue: ", robot.sensorCol1.blue());

        telemetry.addData("Alpha2: ", robot.sensorCol2.alpha());
        telemetry.addData("Red2: ", robot.sensorCol2.red());
        telemetry.addData("Green2: ", robot.sensorCol2.green());
        telemetry.addData("Blue: ", robot.sensorCol2.blue());

        telemetry.addData("Distance: ", robot.sensordist.getDistance(DistanceUnit.INCH));


    }

}
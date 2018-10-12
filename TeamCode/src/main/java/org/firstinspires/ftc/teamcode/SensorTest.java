package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

//import legacy.FoOHardware;

@TeleOp(name="SensorTest", group="SensorTestBot")
@Disabled
public class SensorTest extends OpMode {

    SensorTestHardware robot = new SensorTestHardware();
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
        //forward/backwards movement .8 lim
        leftPower = gamepad1.left_stick_y * pLim;
        rightPower = gamepad1.right_stick_y * pLim;

        //lower threshold .1
        if (Math.abs(gamepad1.left_stick_y) < tHold) {
            leftPower = 0;
        }
        if (Math.abs(gamepad1.right_stick_y) < tHold) {
            rightPower = 0;
        }

        //set the powers
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);

        telemetry.addData("Left-Y", gamepad1.left_stick_y);
        telemetry.addData("Right-Y", gamepad1.right_stick_y);
        telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", robot.sensorDis.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha", robot.sensorCol.alpha());
        telemetry.addData("Red  ", robot.sensorCol.red());
        telemetry.addData("Green", robot.sensorCol.green());
        telemetry.addData("Blue ", robot.sensorCol.blue());
        telemetry.addData("MagnetTouch", robot.sensorMag.getState());
    }

}
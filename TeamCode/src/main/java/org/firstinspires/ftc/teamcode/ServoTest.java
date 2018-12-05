package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import legacy.FoOHardware;

@TeleOp(name="ServoTest", group="Testing")
//@Disabled
public class ServoTest extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runTime = new ElapsedTime();

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Servo: ", robot.bucket.getPosition());


    }

}
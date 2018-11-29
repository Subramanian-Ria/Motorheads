package org.firstinspires.ftc.teamcode.uselessjunk;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import legacy.FoOHardware;

@TeleOp(name="TankHomecomingTeleop", group="TankTestBot")
@Disabled
public class TankHomecomingTeleop extends OpMode {

    TankTestHardware robot = new TankTestHardware();
    ElapsedTime runTime = new ElapsedTime();


    float RPower = 0; //motor power for future use
    float LPower = 0;

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
        LPower = gamepad1.left_stick_y * pLim;
        RPower = gamepad2.right_stick_y * pLim;

        //lower threshold .1
        if (Math.abs(gamepad1.left_stick_y) < tHold) {
            LPower = 0;
        }
        if (Math.abs(gamepad2.right_stick_y) < tHold) {
            RPower = 0;
        }

        //set the powers
        robot.leftMotor.setPower(LPower);
        robot.rightMotor.setPower(RPower);

        telemetry.addData("Left Motor Power", LPower);
        telemetry.addData("Right Motor Power", RPower);
    }

}
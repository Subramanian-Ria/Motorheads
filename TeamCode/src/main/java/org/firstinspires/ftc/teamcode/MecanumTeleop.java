package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

//import legacy.FoOHardware;

@TeleOp(name="MecanumTeleop", group="MecanumBot")
//@Disabled
public class MecanumTeleop extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runTime = new ElapsedTime();
    
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
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.fLMotor.setPower(v1);
        robot.fRMotor.setPower(v2);
        robot.bLMotor.setPower(v3);
        robot.bRMotor.setPower(v4);
    }



}
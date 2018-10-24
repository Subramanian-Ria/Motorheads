package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumTankTeleop", group="MecanumBot")
//@Disabled

public class MecanumTankTeleop extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runTime = new ElapsedTime();
    
    private float pLim = 1f; //power multiplier that acts as a limit
    private float tHold = .1f; //lowest threshold for it to register

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //variables
        float FL;
        float BL;
        float FR;
        float BR;

        //assignment to allow for standard tank drive and strafing
        FL = gamepad1.left_stick_y + gamepad1.left_stick_x; //TODO: CHECK +- ON ROBOT
        BL = gamepad1.left_stick_y - gamepad1.left_stick_x;
        FR = gamepad1.right_stick_y - gamepad1.left_stick_x;
        BR = gamepad1.right_stick_y + gamepad1.left_stick_x;

        //ensures values stay within -pLim and pLim
        FL = Range.clip(FL, -pLim, pLim);
        BL = Range.clip(BL, -pLim, pLim);
        FR = Range.clip(FR, -pLim, pLim);
        BR = Range.clip(BR, -pLim, pLim);

        //ensures values do not fall between -tHold and tHold
        if(Math.abs(FL) < tHold) {
            FL = 0;
        }
        if(Math.abs(BL) < tHold) {
            BL = 0;
        }
        if(Math.abs(FR) < tHold) {
            FR = 0;
        }
        if(Math.abs(BR) < tHold) {
            BR = 0;
        }

        //horizontal arm movement
        /*while(gamepad1.dpad_right) {
            robot.armEx.setPower(pLim);
        }
        robot.armEx.setPower(0);
        while(gamepad1.dpad_left) {
            robot.armEx.setPower(-pLim);
        }
        robot.armEx.setPower(0);

        //intake control
        while(gamepad1.a) {
            robot.intake.setPower(pLim);
        }
        robot.intake.setPower(0);*/

        //assigns powers for driving
        robot.fLMotor.setPower(FL);
        robot.bLMotor.setPower(BL);
        robot.fRMotor.setPower(FR);
        robot.bRMotor.setPower(BR);

        //TODO: RESEARCH INTO BETTER CONTROL SCHEMES TO ALLOW FOR DIAGONAL MOVEMENT (POST SCRIMMAGE)
        //TODO: EVALUATE VIABILITY OF TANK DRIVE OVER THE LONG TERM
        //TODO: LOOK INTO PID BASED CONTROL SYSTEM
    }



}
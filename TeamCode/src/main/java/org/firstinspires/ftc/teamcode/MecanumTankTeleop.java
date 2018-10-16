package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumTankTeleop", group="MecanumBot")
@Disabled

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
        FL = gamepad1.left_stick_y - gamepad1.left_stick_x;
        BL = gamepad1.left_stick_y + gamepad1.left_stick_x;
        FR = gamepad1.right_stick_y + gamepad1.left_stick_x;
        BR = gamepad1.right_stick_y - gamepad1.left_stick_x;

        //ensures values stay within -pLim and pLim
        FL = Range.clip(FL, -pLim, pLim);
        BL = Range.clip(BL, -pLim, pLim);
        FR = Range.clip(FR, -pLim, pLim);
        BR = Range.clip(BR, -pLim, pLim);

        //ensures values do not fall between -tHold and tHold
        if(-tHold < FL && FL < tHold) {
            FL = 0;
        }
        if(-tHold < BL && BL < tHold) {
            BL = 0;
        }
        if(-tHold < FR && FR < tHold) {
            FR = 0;
        }
        if(-tHold < BR && BR < tHold) {
            BR = 0;
        }

        //assigns powers
        robot.fLMotor.setPower(FL);
        robot.bLMotor.setPower(BL);
        robot.fRMotor.setPower(FR);
        robot.bRMotor.setPower(BR);

        //TODO: RESEARCH INTO BETTER CONTROL SCHEMES TO ALLOW FOR DIAGONAL MOVEMENT
        //TODO: EVALUATE VIABILITY OF TANK DRIVE OVER THE LONG TERM
        //TODO: LOOK INTO PID BASED CONTROL SYSTEM
    }



}
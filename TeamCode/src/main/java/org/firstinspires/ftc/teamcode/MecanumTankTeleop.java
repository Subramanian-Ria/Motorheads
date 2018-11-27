package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumTankTeleop", group="MecanumBot")
//@Disabled

public class MecanumTankTeleop extends OpMode {

    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private float pLim = 1f; //power multiplier that acts as a limit
    private float drive = .6f;
    private float tHold = .1f; //lowest threshold for it to register
    private float pSlow = .2f;

    //boolean intakeFor = false;
    //boolean intakeBack = false;

    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.intake.setPower(0);
    }

    @Override
    public void loop() {
        float FL;
        float BL;
        float FR;
        float BR;

        //assignment to allow for standard tank drive and strafing
        FL = gamepad1.right_stick_y + gamepad1.left_stick_x;
        BL = gamepad1.right_stick_y - gamepad1.left_stick_x;
        FR = gamepad1.left_stick_y - gamepad1.left_stick_x;
        BR = gamepad1.left_stick_y + gamepad1.left_stick_x;

        //ensures values stay within -pLim and pLim
        FL = Range.clip(FL, -drive, drive);
        BL = Range.clip(BL, -drive, drive);
        FR = Range.clip(FR, -drive, drive);
        BR = Range.clip(BR, -drive, drive);

        //ensures values do not fall between -tHold and tHold
        if (Math.abs(FL) < tHold) {
            FL = 0;
        }
        if (Math.abs(BL) < tHold) {
            BL = 0;
        }
        if (Math.abs(FR) < tHold) {
            FR = 0;
        }
        if (Math.abs(BR) < tHold) {
            BR = 0;
        }



        //horizontal arm movement
        while(gamepad1.dpad_right) {
            moveMotor(robot.armEx, -pLim);
        }
        robot.armEx.setPower(0);
        while(gamepad1.dpad_left) {
            moveMotor(robot.armEx, pLim);
        }
        robot.armEx.setPower(0);

        //intake control
        if(gamepad1.x) {
            robot.intake.setPower(pLim);
        }
        else if(gamepad1.b) {
            robot.intake.setPower(-.8);
        }
        else if(gamepad1.y) {
            robot.intake.setPower(0);
        }

        //elevator controls
        while(gamepad1.dpad_up) {
            moveMotor(robot.elevator, -1);
        }
        robot.elevator.setPower(0);
        while(gamepad1.dpad_down) {
            moveMotor(robot.elevator, 1);
        }
        robot.elevator.setPower(0);

        //arm flip controls
        /*if(gamepad1.a) {
            encoderMove(robot.armFlip, 11, 20, 0, pLim);
        }
        if(gamepad1.y) {
            encoderMove(robot.armFlip, -10, 20, robot.armFlip.getCurrentPosition(), -pLim);
        }*/

        //manual movement of the arm flip- may replace encoder movement
        while(gamepad1.left_bumper) {
            moveMotor(robot.armFlip, pSlow);
        }
        robot.armFlip.setPower(0);
        while(gamepad1.right_bumper) {
                moveMotor(robot.armFlip, -pSlow);
        }
        robot.armFlip.setPower(0);

        //faster arm movement
        while(gamepad1.left_trigger > tHold) {
            moveMotor(robot.armFlip, pLim);
        }
        robot.armFlip.setPower(0);
        while(gamepad1.right_trigger > tHold) {
            moveMotor(robot.armFlip, -pLim);
        }
        robot.armFlip.setPower(0);

        robot.fLMotor.setPower(FL);
        robot.bLMotor.setPower(BL);
        robot.fRMotor.setPower(FR);
        robot.bRMotor.setPower(BR);
    }
    public void moveMotor(DcMotor motor, float power) {
        motor.setPower(power);
    }
}




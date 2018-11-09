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
    private float tHold = .1f; //lowest threshold for it to register
    private float pSlow = .2f;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        /*if((gamepad1.left_stick_x != 0f) || (gamepad1.left_stick_y != 0) || (gamepad1.right_stick_x != 0) || (gamepad1.right_stick_y)) {

        }*/

        //horizontal arm movement
        while(gamepad1.dpad_right) {
            robot.armEx.setPower(pLim);
            mecanumMove();
        }
        robot.armEx.setPower(0);
        while(gamepad1.dpad_left) {
            robot.armEx.setPower(-pLim);
            mecanumMove();
        }
        robot.armEx.setPower(0);

        //intake control
        while(gamepad1.b) {
            robot.intake.setPower(pLim);
            mecanumMove();
        }
        robot.intake.setPower(0);
        while(gamepad1.x) {
            robot.intake.setPower(-pLim);
            mecanumMove();
        }
        robot.intake.setPower(0);

        //elevator controls
        while(gamepad1.dpad_up) {
            robot.elevator.setPower(1);
            mecanumMove();
        }
        robot.elevator.setPower(0);
        while(gamepad1.dpad_down) {
            robot.elevator.setPower(-1);
            mecanumMove();
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
            robot.armFlip.setPower(-pSlow);
            mecanumMove();
        }
        robot.armFlip.setPower(0);
        while(gamepad1.right_bumper) {
            robot.armFlip.setPower(pSlow);
            mecanumMove();
        }
        robot.armFlip.setPower(0);

        //faster arm movement
        while(gamepad1.left_trigger > tHold) {
            robot.armFlip.setPower(-pLim);
            mecanumMove();
        }
        robot.armFlip.setPower(0);
        while(gamepad1.right_trigger > tHold) {
            robot.armFlip.setPower(pLim);
            mecanumMove();
        }
        robot.armFlip.setPower(0);

        mecanumMove();
    }
    public void mecanumMove() {
        //variables
        float FL;
        float BL;
        float FR;
        float BR;

        //assignment to allow for standard tank drive and strafing
        FL = gamepad1.left_stick_y + gamepad1.left_stick_x;
        BL = gamepad1.left_stick_y - gamepad1.left_stick_x;
        FR = gamepad1.right_stick_y - gamepad1.left_stick_x;
        BR = gamepad1.right_stick_y + gamepad1.left_stick_x;

        //ensures values stay within -pLim and pLim
        FL = Range.clip(FL, -pLim, pLim);
        BL = Range.clip(BL, -pLim, pLim);
        FR = Range.clip(FR, -pLim, pLim);
        BR = Range.clip(BR, -pLim, pLim);

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

        robot.fLMotor.setPower(-FL);
        robot.bLMotor.setPower(BL);
        robot.fRMotor.setPower(-FR);
        robot.bRMotor.setPower(-BR);
    }

    /*public void encoderMove(DcMotor motor, double inches, double timeoutS, int ref, float power) {
        // Determine new target position, and pass to motor controller
        int target = ref + (int) (inches * COUNTS_PER_INCH);
        motor.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        motor.setPower(power);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() <= timeoutS) && robot.armFlip.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", target);
            telemetry.addData("Path2", "Running at %7d", motor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        motor.setPower(0);

        // Turn off RUN_TO_POSITION
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/
}




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

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder TODO: CHECK THIS
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

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
        /*while(gamepad1.dpad_right) {
            robot.armEx.setPower(pLim);
        }
        robot.armEx.setPower(0);
        while(gamepad1.dpad_left) {
            robot.armEx.setPower(-pLim);
        }
        robot.armEx.setPower(0);

        //intake control
        while(gamepad1.b) {
            robot.intake.setPower(pLim);
        }
        robot.intake.setPower(0);*/

        /*while(gamepad1.dpad_up) {
            robot.elevator.setPower(pLim);
        }
        robot.elevator.setPower(0);
        while(gamepad1.dpad_down) {
            robot.elevator.setPower(-pLim);
        }
        robot.elevator.setPower(0);*/

        /*if(gamepad1.a) {
            encoderMove(12, 5); //TODO: CHECK VALUES & SEE IF IT EVEN WORKS
        }*/

        //assigns powers for driving
        robot.fLMotor.setPower(FL);
        robot.bLMotor.setPower(BL);
        robot.fRMotor.setPower(FR);
        robot.bRMotor.setPower(BR);

        //TODO: RESEARCH INTO BETTER CONTROL SCHEMES TO ALLOW FOR DIAGONAL MOVEMENT (POST SCRIMMAGE)
        //TODO: EVALUATE VIABILITY OF TANK DRIVE OVER THE LONG TERM
        //TODO: LOOK INTO PID BASED CONTROL SYSTEM
    }

    public void encoderMove(double inches, double timeoutS) {
        int target;

        // Determine new target position, and pass to motor controller
        target = robot.armFlip.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        robot.armFlip.setTargetPosition(target);

        // Turn On RUN_TO_POSITION
        robot.armFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.armFlip.setPower(Math.abs(pLim));//math.abs for armFlip?? TODO:TEST ON ARM FLIP TO SEE IF IT NEEDS TO BE CHANGED

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() <= timeoutS) && robot.armFlip.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", target);
            telemetry.addData("Path2", "Running at %7d", robot.armFlip.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.armFlip.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.armFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(250); optional pause after each move


    }
}




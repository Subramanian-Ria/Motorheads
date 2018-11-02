package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="ArmTestTeleop", group="ArmTestBot")
@Disabled

public class ArmTestTeleop extends OpMode {

    ArmTestHardware robot = new ArmTestHardware();
    ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    private float pLim = 1f; //power multiplier that acts as a limit
    private float tHold = .1f; //lowest threshold for it to register
    boolean flipMode = false;

    //reference position for the arm flip

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
    }


    @Override
    public void loop() {
        //horizontal arm movement
        while(gamepad1.dpad_right) {
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
        robot.intake.setPower(0);
        while(gamepad1.x) {
            robot.intake.setPower(-pLim);
        }
        robot.intake.setPower(0);

        //elevator controls
        while(gamepad1.dpad_up) {
            encoderMove(robot.elevator, 100, 20, 0, pLim);
        }
        while(gamepad1.dpad_down) {
            encoderMove(robot.elevator, -100 , 20, 0, -pLim);
        }

        //arm flip controls
        if(gamepad1.a) {
            encoderMove(robot.armFlip, 11, 20, 0, pLim);
        }
        if(gamepad1.y) {
            encoderMove(robot.armFlip, -10, 20, 0, -pLim);//TODO: CHECK REF VALUE
        }
    }

    public void encoderMove(DcMotor motor, double inches, double timeoutS, int ref, float power) {
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
    }
}




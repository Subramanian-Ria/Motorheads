package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MecanumTeleop", group="MecanumBot")
//@Disabled

public class MecanumTeleop extends OpMode {

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
    private float pSlowSlow = .1f;

    //boolean intakeFor = false;
    boolean intakeOld = true;
    boolean intakeNew = false;
    boolean PowerOn=true;
    int armFlipRef = 0;//TODO: FIND VALUE
    int count = 0;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.intake.setPower(0);
        robot.armFlip.setPower(0);
    }

    @Override
    public void loop() {
        //GAMEPAD1 TELE-OP
        //horizontal arm movement
        /*if (gamepad1.dpad_right) {
            robot.armEx.setPower(-pLim);
        }
        else if (gamepad1.dpad_left) {
            robot.armEx.setPower(pLim);
        }
        else {
            robot.armEx.setPower(0);
        }

        if(gamepad1.y) {
            robot.bucket.setPosition(.6);//flat position
        }
        if(gamepad1.a) {
            robot.bucket.setPosition(.45);//cube position
        }
        if(gamepad1.b) {
            robot.bucket.setPosition(.25);//armFlip position
        }
        //if(gamepad1.x) {
        intakeNew= gamepad1.x;
        if(intakeNew != intakeOld)
        {
            PowerOn= !PowerOn;
        }
        if(PowerOn)
        {
            robot.intake.setPower(0.6);
        }
        else
        {
            robot.intake.setPower(0);
        }
        intakeOld = intakeNew;


        //elevator controls
        if (gamepad1.dpad_up) {
            robot.elevator.setPower(-1);
        }
        else if (gamepad1.dpad_down) {
            robot.elevator.setPower(1);
        }
        else {
            robot.elevator.setPower(0);
        }

        //manual movement of the arm flip- may replace encoder movement
        if (gamepad1.left_bumper) {
            robot.armFlip.setPower(-pSlowSlow);
        }
        else if (gamepad1.right_bumper) {
            robot.armFlip.setPower(pSlowSlow);
        }

        //fast arm flip
        else if (gamepad1.left_trigger > tHold)
        {
            robot.armFlip.setPower(-pLim);
        }
        else if (gamepad1.right_trigger > tHold) {
            robot.armFlip.setPower(pLim);
        }
        else {
            robot.armFlip.setPower(0);
        }*/
        mecanumMove();
        //
        //
        //
        //
        //
        //
        //
        //GAMEPAD2 TELE-OP
        //horizontal arm movement
        if (gamepad2.dpad_right) {
            robot.armEx.setPower(-pLim);
        }
        else if (gamepad2.dpad_left) {
            robot.armEx.setPower(pLim);
        }
        else {
            robot.armEx.setPower(0);
        }

        //3 pos bucket
        if(gamepad2.y) {
            robot.bucket.setPosition(.6);//flat position
        }
        if(gamepad2.a) {
            robot.bucket.setPosition(.45);//cube position
        }
        if(gamepad2.b) {
            robot.bucket.setPosition(.25);//armFlip position
        }
        //if(gamepad1.x) {
        //intake on/off using gamepad x
        intakeNew= gamepad2.x;
        if(intakeNew != intakeOld)
        {
            PowerOn= !PowerOn;
        }
        if(PowerOn)
        {
            robot.intake.setPower(0.6);
        }
        else
        {
            robot.intake.setPower(0);
        }
        intakeOld = intakeNew;

        //elevator controls
        if (gamepad2.dpad_up) {
            robot.elevator.setPower(-1);
        }
        else if (gamepad2.dpad_down) {
            robot.elevator.setPower(1);
        }
        else {
            robot.elevator.setPower(0);
        }
        //manual movement of the arm flip- may replace encoder movement
        if (gamepad2.left_bumper) {
            robot.armFlip.setPower(-pSlowSlow);

        }
        else if (gamepad2.right_bumper) {
            robot.armFlip.setPower(pSlowSlow);
        }

        //fast arm flip
        else if (gamepad2.left_trigger > tHold)
        {
            robot.armFlip.setPower(-pLim);
        }
        else if (gamepad2.right_trigger > tHold) {
            robot.armFlip.setPower(pLim);
        }
        else {
            robot.armFlip.setPower(0);
        }
        //mecanumMove();
    }

    public void mecanumMove() {
        //variables
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.fLMotor.setPower(-drive * v1);
        robot.fRMotor.setPower(-drive * v2);
        robot.bLMotor.setPower(-drive * v3);
        robot.bRMotor.setPower(-drive * v4);


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
}
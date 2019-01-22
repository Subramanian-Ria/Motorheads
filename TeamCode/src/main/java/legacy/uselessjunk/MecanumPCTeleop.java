package legacy.uselessjunk;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import legacy.servoLeftTest;


@TeleOp(name="MecanumPCTeleop", group="MecanumBot")
@Disabled

public class MecanumPCTeleop extends OpMode {

    servoLeftTest.MecanumHardware robot = new servoLeftTest.MecanumHardware();
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

        //horizontal arm movement
        if (gamepad1.dpad_right) {
            robot.armEx.setPower(-pLim);
        }
        if (gamepad1.dpad_left) {
            robot.armEx.setPower(pLim);
        }


        //intake control
        if (gamepad1.x) {
            robot.intake.setPower(pLim);
        } else if (gamepad1.b) {
            robot.intake.setPower(-.8);
        } else if (gamepad1.y) {
            robot.intake.setPower(0);
        }

        //elevator controls
        if (gamepad1.dpad_up) {
            robot.elevator.setPower(-1);
        }
        if (gamepad1.dpad_down) {
            robot.elevator.setPower(1);
        }


        //arm flip controls
        /*if(gamepad1.a) {
            encoderMove(robot.armFlip, 11, 20, 0, pLim);
        }
        if(gamepad1.y) {
            encoderMove(robot.armFlip, -10, 20, robot.armFlip.getCurrentPosition(), -pLim);
        }*/

        //manual movement of the arm flip- may replace encoder movement
        if (gamepad1.left_bumper) {
            //runtime.reset();
            //while(runtime.seconds() < 2) {
            //robot.armFlip.setPower(-pSlow);
            //mecanumMove();`
            //}
            robot.armFlip.setPower(-pSlow);
        }
        if (gamepad1.right_bumper) {
            //runtime.reset();
            //while(runtime.seconds() < 2) {
            robot.armFlip.setPower(pSlow);
            //}
            //robot.armFlip.setPower(0);
            //mecanumMove();
        }


        //faster arm movement
        if (gamepad1.left_trigger > tHold) {
            robot.armFlip.setPower(-pLim);
        }
        if (gamepad1.right_trigger > tHold) {
            robot.armFlip.setPower(pLim);
        }
        robot.armFlip.setPower(0);
        robot.elevator.setPower(0);
        robot.armEx.setPower(0);
        mecanumMove();
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

        robot.fLMotor.setPower(v1);
        robot.fRMotor.setPower(v2);
        robot.bLMotor.setPower(v3);
        robot.bRMotor.setPower(v4);


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
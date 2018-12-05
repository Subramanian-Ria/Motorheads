

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="ElevatorLowerWithClicks", group="MecanumBot")
//@Disabled
public class ElevatorLowerWithClicks extends LinearOpMode{

    /* Declare OpMode members. */
    MecanumHardware         robot   = new MecanumHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double COUNTS_PER_REV    = 1120 ;// Andymark 40...  TETRIX Motor Encoder = 1440
    static final double DMT       = 3.98;     // Diameter of the wheel
    static final double COUNTS_PER_INCH = COUNTS_PER_REV * Math.PI * DMT;
    static final double DRIVE_SPEED             = 0.6;
    static final double TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                robot.elevator.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderElevator(1, -9.6,40);
        telemetry.update();
   //     encoderTurn(TURN_SPEED, 4.0
        // telemetry.addData("Turn", "Complete");
        //telemetry.update();
    }

    /*
     This is the method to drive straight either forward or backward
     */
    public void encoderDrive(double FLInch, double FRInch, double BLInch, double BRInch, double timeoutS, double Speed)
    {

        int TargetFL;
        int TargetFR;
        int TargetBL;
        int TargetBR;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            TargetFL = robot.fLMotor.getCurrentPosition() + (int)( FLInch* COUNTS_PER_INCH);
            TargetFR = robot.fRMotor.getCurrentPosition() + (int)( FRInch* COUNTS_PER_INCH);
            TargetBL = robot.fRMotor.getCurrentPosition() + (int)( BLInch* COUNTS_PER_INCH);
            TargetBR = robot.fRMotor.getCurrentPosition() + (int)( BRInch* COUNTS_PER_INCH);

            robot.fLMotor.setTargetPosition(TargetFL);
            robot.fRMotor.setTargetPosition(TargetFR);
            robot.bRMotor.setTargetPosition(TargetFR);
            robot.bLMotor.setTargetPosition(TargetFR);


            // Turn On RUN_TO_POSITION
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.fLMotor.setPower(Math.abs(Speed));
            robot.fRMotor.setPower(Math.abs(Speed));
            robot.bRMotor.setPower(Math.abs(Speed));
            robot.bLMotor.setPower(Math.abs(Speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
            {

                //Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetBL,  TargetBR, TargetFL, TargetFR);

                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.bRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.bLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.fLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderElevator(double speed,double distance, double timeoutS) {
        int newElevatorTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(distance*COUNTS_PER_REV);
            robot.elevator.setTargetPosition(newElevatorTarget);

            // Turn On RUN_TO_POSITION
            robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.elevator.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.elevator.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newElevatorTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.elevator.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.elevator.setPower(0);

            // Reset encoders
            robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
//        public void encoderTurn(double speed,double timeoutS) {
//            int newLeftTarget;
//            int newRightTarget;
//
//            // Ensure that the opmode is still active
//            if (opModeIsActive()) {
//
//                // Determine new target position, and pass to motor controller
//                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(CLICKS);
//                newRightTarget = robot.rightDrive.getCurrentPosition() - (int)(CLICKS);
//                robot.leftDrive.setTargetPosition(newLeftTarget);
//                robot.rightDrive.setTargetPosition(newRightTarget);
//
//                // Turn On RUN_TO_POSITION
//                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                // reset the timeout time and start motion.
//                runtime.reset();
//                robot.leftDrive.setPower(Math.abs(speed));
//                robot.rightDrive.setPower(Math.abs(speed));
//
//                // keep looping while we are still active, and there is time left, and both motors are running.
//                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//                // its target position, the motion will stop.  This is "safer" in the event that the robot will
//                // always end the motion as soon as possible.
//                // However, if you require that BOTH motors have finished their moves before the robot continues
//                // onto the next step, use (isBusy() || isBusy()) in the loop test.
//                while (opModeIsActive() &&
//                        (runtime.seconds() < timeoutS) ||
//                        (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
//
//                    // Display it for the driver.
//                    telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                    telemetry.addData("Path2",  "Running at %7d :%7d",
//                            robot.leftDrive.getCurrentPosition(),
//                            robot.rightDrive.getCurrentPosition());
//                    telemetry.update();
//                }
//
//                // Stop all motion;
//                robot.leftDrive.setPower(0);
//                robot.rightDrive.setPower(0);
//
//                // Reset encoders
//                robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                //  sleep(250);   // optional pause after each move
//            }
//        }

    @Autonomous(name="IntakeTestAuton", group="MecanumBot")
    //@Disabled
    public static class IntakeTestAuton extends LinearOpMode{

        /* Declare OpMode members. */
        MecanumHardware robot   = new MecanumHardware();   // Use a Pushbot's hardware
        private ElapsedTime     runtime = new ElapsedTime();

        static final double     CLICKS    = 1120 ;    // Andymark 40...  TETRIX Motor Encoder = 1440
        static final double     DMT       = 3.98;     // Diameter of the wheel
        static final double     DRIVE_SPEED             = 0.6;
        static final double     TURN_SPEED              = 0.5;

        @Override
        public void runOpMode() {

            /*
             * Initialize the drive system variables.
             * The init() method of the hardware class does all the work here
             */
            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
         //robot.intake.setPower(-1);
         sleep(1000);
         //robot.intake.setPower(0);
        }
        public void setDrivePower(double power) {
            robot.bLMotor.setPower(power);
            robot.bRMotor.setPower(power);
            robot.fRMotor.setPower(power);
            robot.fLMotor.setPower(power);
        }

        /*
         This is the method to drive straight either forward or backward
         */

        }
}




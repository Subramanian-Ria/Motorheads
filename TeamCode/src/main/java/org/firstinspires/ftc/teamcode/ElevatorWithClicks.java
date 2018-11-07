

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="EncodersWithClicks", group="Pushbot")
//@Disabled
public class ElevatorWithClicks extends LinearOpMode{

    /* Declare OpMode members. */
    MecanumHardware         robot   = new MecanumHardware();   // Use a Pushbot's hardware
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
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.elevator.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 18,5);
        telemetry.update();
   //     encoderTurn(TURN_SPEED, 4.0);
        telemetry.addData("Turn", "Complete");
        telemetry.update();
    }

    /*
     This is the method to drive straight either forward or backward
     */
    public void encoderDrive(double speed,double distance, double timeoutS) {
        int newElevatorTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newElevatorTarget = robot.elevator.getCurrentPosition() + (int)(3*CLICKS);
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newElevatorTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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
}




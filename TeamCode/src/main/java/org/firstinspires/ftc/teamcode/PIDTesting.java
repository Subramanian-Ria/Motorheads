// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="PID Testing", group="Testing")
//@Disabled
public class PIDTesting extends LinearOpMode {
    MecanumHardware robot = new MecanumHardware();
    ElapsedTime runTime = new ElapsedTime();
    //DcMotor leftMotor;
    //DcMotor rightMotor;
    DigitalChannel touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation angles;
    Acceleration gravity;

    //turn headings
    public float NORTH;
    public float EAST;
    public float WEST;
    public float SOUTH;

    static final double     COUNTS_PER_MOTOR_REV = 1120 ;    // Currently: Andymark Neverest 40
    static final double     DRIVE_GEAR_REDUCTION = 2.0 ;     // This is < 1.0 if geared UP //On OUR CENTER MOTOR THE GEAR REDUCTION IS .5
    static final double     DRIVE_GEAR_REDUCTION_CM = 0.5 ;
    static final double     WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    static final double     COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_INCH_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION_CM) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED = .4;
    static final double TURN_SPEED = .11;
    double globalAngle, power = .30, correction;
    //boolean aButton, bButton, touched;
    PIDController pidRotate, pidDrive;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // drive until end of period.

        while (opModeIsActive()) {
            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            // set power levels.
            robot.fLMotor.setPower(-power + correction);
            robot.bLMotor.setPower(-power + correction);
            robot.fRMotor.setPower(-power);
            robot.bRMotor.setPower(-power);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            if (gamepad1.a || gamepad1.b) {
                // backup.
                robot.fLMotor.setPower(power);
                robot.bLMotor.setPower(power);
                robot.fRMotor.setPower(power);
                robot.bRMotor.setPower(power);

                sleep(500);

                // stop.
                robot.fLMotor.setPower(0);
                robot.bLMotor.setPower(0);
                robot.fRMotor.setPower(0);
                robot.bRMotor.setPower(0);

                // turn 90 degrees right.
                if (gamepad1.a) rotate(-90, power);

                // turn 90 degrees left.
                if (gamepad1.b) rotate(90, power);
            }
        }

        // turn the motors off.
        robot.fLMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bRMotor.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle with a minimum of 20%.
        // This is to prevent the robots momentum from overshooting the turn after we turn off the
        // power. The PID controller reports onTarget() = true when the difference between turn
        // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
        // The minimum power is determined by testing and must enough to prevent motor stall and
        // complete the turn. Note: if the gap between the starting power and the stall (minimum)
        // power is small, overshoot may still occur. Overshoot is dependant on the motor and
        // gearing configuration, starting power, weight of the robot and the on target tolerance.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(.20, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                robot.fLMotor.setPower(-power);
                robot.bLMotor.setPower(-power);
                robot.fRMotor.setPower(power);
                robot.bRMotor.setPower(power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.fLMotor.setPower(power);
                robot.bLMotor.setPower(power);
                robot.fRMotor.setPower(-power);
                robot.bRMotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.fLMotor.setPower(power);
                robot.bLMotor.setPower(power);
                robot.fRMotor.setPower(-power);
                robot.bRMotor.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.fLMotor.setPower(0);
        robot.bLMotor.setPower(0);
        robot.fRMotor.setPower(0);
        robot.bRMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    // PID controller courtesy of Peter Tischler, with modifications.

    public class PIDController {
        private double m_P;                                 // factor for "proportional" control
        private double m_I;                                 // factor for "integral" control
        private double m_D;                                 // factor for "derivative" control
        private double m_input;                 // sensor input for pid controller
        private double m_maximumOutput = 1.0;   // |maximum output|
        private double m_minimumOutput = -1.0;  // |minimum output|
        private double m_maximumInput = 0.0;    // maximum input - limit setpoint to this
        private double m_minimumInput = 0.0;    // minimum input - limit setpoint to this
        private boolean m_continuous = false;   // do the endpoints wrap around? eg. Absolute encoder
        private boolean m_enabled = false;              //is the pid controller enabled
        private double m_prevError = 0.0;           // the prior sensor input (used to compute velocity)
        private double m_totalError = 0.0;      //the sum of the errors for use in the integral calc
        private double m_tolerance = 0.05;          //the percentage error that is considered on target
        private double m_setpoint = 0.0;
        private double m_error = 0.0;
        private double m_result = 0.0;

        /**
         * Allocate a PID object with the given constants for P, I, D
         *
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd) {
            m_P = Kp;
            m_I = Ki;
            m_D = Kd;
        }

        /**
         * Read the input, calculate the output accordingly, and write to the output.
         * This should only be called by the PIDTask
         * and is created during initialization.
         */
        private void calculate() {
            int sign = 1;

            // If enabled then proceed into controller calculations
            if (m_enabled) {
                // Calculate the error signal
                m_error = m_setpoint - m_input;

                // If continuous is set to true allow wrap around
                if (m_continuous) {
                    if (Math.abs(m_error) > (m_maximumInput - m_minimumInput) / 2) {
                        if (m_error > 0)
                            m_error = m_error - m_maximumInput + m_minimumInput;
                        else
                            m_error = m_error + m_maximumInput - m_minimumInput;
                    }
                }

                // Integrate the errors as long as the upcoming integrator does
                // not exceed the minimum and maximum output thresholds.

                if ((Math.abs(m_totalError + m_error) * m_I < m_maximumOutput) &&
                        (Math.abs(m_totalError + m_error) * m_I > m_minimumOutput))
                    m_totalError += m_error;

                // Perform the primary PID calculation
                m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

                // Set the current error to the previous error for the next cycle.
                m_prevError = m_error;

                if (m_result < 0) sign = -1;    // Record sign of result.

                // Make sure the final result is within bounds. If we constrain the result, we make
                // sure the sign of the constrained result matches the original result sign.
                if (Math.abs(m_result) > m_maximumOutput)
                    m_result = m_maximumOutput * sign;
                else if (Math.abs(m_result) < m_minimumOutput)
                    m_result = m_minimumOutput * sign;
            }
        }

        /**
         * Set the PID Controller gain parameters.
         * Set the proportional, integral, and differential coefficients.
         *
         * @param p Proportional coefficient
         * @param i Integral coefficient
         * @param d Differential coefficient
         */
        public void setPID(double p, double i, double d) {
            m_P = p;
            m_I = i;
            m_D = d;
        }

        /**
         * Get the Proportional coefficient
         *
         * @return proportional coefficient
         */
        public double getP() {
            return m_P;
        }

        /**
         * Get the Integral coefficient
         *
         * @return integral coefficient
         */
        public double getI() {
            return m_I;
        }

        /**
         * Get the Differential coefficient
         *
         * @return differential coefficient
         */
        public double getD() {
            return m_D;
        }

        /**
         * Return the current PID result for the last input set with setInput().
         * This is always centered on zero and constrained the the max and min outs
         *
         * @return the latest calculated output
         */
        public double performPID() {
            calculate();
            return m_result;
        }

        /**
         * Return the current PID result for the specified input.
         *
         * @param input The input value to be used to calculate the PID result.
         *              This is always centered on zero and constrained the the max and min outs
         * @return the latest calculated output
         */
        public double performPID(double input) {
            setInput(input);
            return performPID();
        }

        /**
         * Set the PID controller to consider the input to be continuous,
         * Rather then using the max and min in as constraints, it considers them to
         * be the same point and automatically calculates the shortest route to
         * the setpoint.
         *
         * @param continuous Set to true turns on continuous, false turns off continuous
         */
        public void setContinuous(boolean continuous) {
            m_continuous = continuous;
        }

        /**
         * Set the PID controller to consider the input to be continuous,
         * Rather then using the max and min in as constraints, it considers them to
         * be the same point and automatically calculates the shortest route to
         * the setpoint.
         */
        public void setContinuous() {
            this.setContinuous(true);
        }

        /**
         * Sets the maximum and minimum values expected from the input.
         *
         * @param minimumInput the minimum value expected from the input, always positive
         * @param maximumInput the maximum value expected from the output, always positive
         */
        public void setInputRange(double minimumInput, double maximumInput) {
            m_minimumInput = Math.abs(minimumInput);
            m_maximumInput = Math.abs(maximumInput);
            setSetpoint(m_setpoint);
        }

        /**
         * Sets the minimum and maximum values to write.
         *
         * @param minimumOutput the minimum value to write to the output, always positive
         * @param maximumOutput the maximum value to write to the output, always positive
         */
        public void setOutputRange(double minimumOutput, double maximumOutput) {
            m_minimumOutput = Math.abs(minimumOutput);
            m_maximumOutput = Math.abs(maximumOutput);
        }

        /**
         * Set the setpoint for the PIDController
         *
         * @param setpoint the desired setpoint
         */
        public void setSetpoint(double setpoint) {
            int sign = 1;

            if (m_maximumInput > m_minimumInput) {
                if (setpoint < 0) sign = -1;

                if (Math.abs(setpoint) > m_maximumInput)
                    m_setpoint = m_maximumInput * sign;
                else if (Math.abs(setpoint) < m_minimumInput)
                    m_setpoint = m_minimumInput * sign;
                else
                    m_setpoint = setpoint;
            } else
                m_setpoint = setpoint;
        }

        /**
         * Returns the current setpoint of the PIDController
         *
         * @return the current setpoint
         */
        public double getSetpoint() {
            return m_setpoint;
        }

        /**
         * Retruns the current difference of the input from the setpoint
         *
         * @return the current error
         */
        public synchronized double getError() {
            return m_error;
        }

        /**
         * Set the percentage error which is considered tolerable for use with
         * OnTarget. (Input of 15.0 = 15 percent)
         *
         * @param percent error which is tolerable
         */
        public void setTolerance(double percent) {
            m_tolerance = percent;
        }

        /**
         * Return true if the error is within the percentage of the total input range,
         * determined by setTolerance. This assumes that the maximum and minimum input
         * were set using setInputRange.
         *
         * @return true if the error is less than the tolerance
         */
        public boolean onTarget() {
            return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maximumInput - m_minimumInput)));
        }

        /**
         * Begin running the PIDController
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Stop running the PIDController.
         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Reset the previous error,, the integral term, and disable the controller.
         */
        public void reset() {
            disable();
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }

        /**
         * Set the input value to be used by the next call to performPID().
         *
         * @param input Input value to the PID calculation.
         */
        public void setInput(double input) {
            int sign = 1;

            if (m_maximumInput > m_minimumInput) {
                if (input < 0) sign = -1;

                if (Math.abs(input) > m_maximumInput)
                    m_input = m_maximumInput * sign;
                else if (Math.abs(input) < m_minimumInput)
                    m_input = m_minimumInput * sign;
                else
                    m_input = input;
            } else
                m_input = input;
        }
        public void encoderDrive(double inches, String direction, double timeoutS, double Speed)
        {

            int TargetFL = 0;
            int TargetFR = 0;
            int TargetBL = 0;
            int TargetBR = 0;


            String heading = direction;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                if(heading == "f")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);

                }

                else if(heading == "b")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);


                }

                else if(heading == "r")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetBR = robot.bRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); //weird should be +


                }

                else if(heading == "l")
                {
                    TargetFL = robot.fLMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);
                    TargetFR = robot.fRMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH);
                    TargetBL = robot.bLMotor.getCurrentPosition() - (int)( inches* COUNTS_PER_INCH); // weird should be +
                    TargetBR = robot.bRMotor.getCurrentPosition() + (int)( inches* COUNTS_PER_INCH);

                }

                else
                {
                    telemetry.addData("not a valid direction", heading );
                }

                // Determine new target position, and pass to motor controller

                robot.fLMotor.setTargetPosition(TargetFL);
                robot.fRMotor.setTargetPosition(TargetFR);
                robot.bRMotor.setTargetPosition(TargetBR);
                robot.bLMotor.setTargetPosition(TargetBL);


                // Turn On RUN_TO_POSITION
                robot.fLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.fRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.bLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                // reset the timeout time and start motion.
                runTime.reset();
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
                        (runTime.seconds() < timeoutS) && ((robot.fLMotor.isBusy() && robot.fRMotor.isBusy()) && robot.bLMotor.isBusy() && robot.bRMotor.isBusy()))
                {

                    //Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", TargetFL,  TargetFR, TargetBL, TargetBR);

                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", robot.fLMotor.getCurrentPosition(), robot.fRMotor.getCurrentPosition(), robot.bLMotor.getCurrentPosition(), robot.bRMotor.getCurrentPosition());
                    //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
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
        public void setCardinalDir()
        {
            updateAngles();
            NORTH = angles.firstAngle;
            EAST = NORTH + 90;
            if (EAST >= 180)
            {
                EAST -= 360;
            }
            else if (EAST <= -180)
            {
                EAST += 360;
            }
            SOUTH = NORTH + 180;
            if (SOUTH >= 180)
            {
                SOUTH -= 360;
            }
            else if (SOUTH <= -180)
            {
                SOUTH += 360;
            }
            WEST = NORTH - 90;
            if (WEST >= 180)
            {
                WEST -= 360;
            }
            else if (WEST <= 180)
            {
                WEST += 360;
            }
        }
        public void updateAngles()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        public double readAngle(String xyz)
        {
            Orientation angles;
            Acceleration gravity;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(xyz.equals("x")){
                return angles.thirdAngle;
            }else if(xyz.equals("y")){
                return angles.secondAngle;
            }else if(xyz.equals("z")){
                return angles.firstAngle;
            }else{
                return 0;
            }
        }
        public void turnDegrees(double target, double power, double timeoutS)
        {
            //Write code to correct to a target position (NOT FINISHED)
            runTime.reset();
            updateAngles(); //variable for gyro correction around z axis
            target *= -1;//switches clockwise and counterclockwise directions
            if(target > 0) {//this fixes a problem where the turn undershoots by 6ish degrees for some reason
                target += 6;
            }
            else if(target < 0){
                target -= 6;
            }
            //target += 6;
            double error = angles.firstAngle - target;
            double errorAbs;
            //wrapping error to have it remain in the field
            if (error > 180)  error -= 360;
            if (error <= -180) error += 360;

            double powerScaled = power;
            do
            {
                updateAngles();
                error = angles.firstAngle - target;
                errorAbs = Math.abs(error);

                if (errorAbs <= 10)
                {
                    powerScaled /= 2;
                }
                telemetry.addData("error", error);
                telemetry.addData("NORTH", NORTH);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                if(error > 0)
                {
                    robot.fRMotor.setPower(powerScaled);
                    robot.bRMotor.setPower(powerScaled);
                    robot.fLMotor.setPower(-powerScaled);
                    robot.bLMotor.setPower(-powerScaled);
                }
                else if(error < 0)
                {
                    robot.fRMotor.setPower(-powerScaled);
                    robot.bRMotor.setPower(-powerScaled);
                    robot.fLMotor.setPower(powerScaled);
                    robot.bLMotor.setPower(powerScaled);
                }
            }
            while ((Math.abs(error) > 1.5) && (runTime.seconds() < timeoutS) && opModeIsActive());

            robot.fRMotor.setPower(0);
            robot.bRMotor.setPower(0);
            robot.fLMotor.setPower(0);
            robot.bLMotor.setPower(0);
        }
    }
}
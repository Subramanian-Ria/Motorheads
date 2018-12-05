package org.firstinspires.ftc.teamcode.uselessjunk;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="GyroCorrection", group="GyroTest")
@Disabled
public class GyroCorrection extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot2 robot   = new HardwarePushbot2();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    //static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // This is for the AndyMark motors //Use 560 for neverest 20, 1120 for neverest 40
    static final double     COUNTS_PER_MOTOR_REV    = 560 ; //These are for the new motors
    static final double     DRIVE_GEAR_REDUCTION    = 25/16;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 90/25.4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters Gparameters = new BNO055IMU.Parameters();
        Gparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        Gparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        Gparameters.loggingEnabled      = true;
        Gparameters.loggingTag          = "IMU";
        Gparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(Gparameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.servoLeft.setPosition(0.2); //l1 r 0
        //robot.servoRight.setPosition(0.8);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //print the angles at the beginning of the program
        telemetry.addData("X", readAngle("z"));
        telemetry.addData("Y", readAngle("y"));
        telemetry.addData("Z", readAngle("x"));
        telemetry.update();

        gyroZDrive(0.2, 90, 10);
    }

    public void gyroZDrive(double power, double inches, int timeoutSeconds) {
        updateAngles();
        double target = robot.leftDrive.getCurrentPosition() + inches;
        //relTarget (relative target) = target - current
        //  This variable stores how many angles away the target is, relative to the current position
        //ang1 = initial angle
        //angRel = current angle (zeroed at ang1). So true angle - ang1
        //relTarget = -ang1 + target
        //WORKS UNTIL true angle REACHES +/-180
        double ang1 = readAngle("x");
        double angle = readAngle("x");
        double reltarget = target - angle;

        while(opModeIsActive() && ((reltarget >1 ) || (reltarget < -1))) {
            telemetry.addData("ang1", ang1);
            telemetry.addData("angle", angle);
            telemetry.addData("reltarget", reltarget);
            telemetry.addData("X: ", angles.firstAngle);
            telemetry.update();
            angle = readAngle("x");
            reltarget = target - angle;
            if(reltarget < 0) {
                normalDrive(power, -power);
            }
            else if(reltarget > 0) {
                normalDrive(-power, power);
            }
        }
    }

    public void normalDrive(double lpower, double rpower) {
        if (opModeIsActive()) {
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDrive.setPower(lpower);
            robot.rightDrive.setPower(-rpower);
        }
    }

    public double readAngle(String xyz)
    {
        Acceleration gravity;
        updateAngles();
        if(xyz.equals("z")) {
            return angles.thirdAngle;
        }
        else if(xyz.equals("y")){
            return angles.secondAngle;
        }
        else if(xyz.equals("x")) {
            return angles.firstAngle;
        }
        else {
            return 0;
        }
    }
    public void updateAngles()
    {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
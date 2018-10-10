//THIS IS A MOTORHEADS PROGRAM

package legacy;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;


public class ForceofHaskinsHardware
{
    /* Public OpMode members. */
    //Initialize everything
    public DcMotor motorLeft;
    public DcMotor  motorRight;
    public DcMotor motorCenter;

    public DcMotor verticalarm;
    public DcMotor horizontalarm;
    public Servo haServo;
    public Servo vaServoRight;
    public Servo vaServoLeft;
    public Servo haWrist;

    //public Servo vaServoRight = null;
    public ColorSensor sensorColor;
    public ColorSensor floorColor;
    public Servo colorServo;

    public DistanceSensor distanceSensor;

    public Servo touchArm;
    public DigitalChannel touchRed; //to use in red autonomous
    public DigitalChannel touchBlue; //to use in blue autonomous

    //declaring values for use with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 0.19685039;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     VERT_INCHES             = 3.0;      //inches to raise verticalarm       TODO: test for real value
    static final double     HORZ_INCHES             = 3.0;      //inches to extend horzizantalarm   TODO: test for real value
    static final double     EN_ARM_SPEED            = 0.2;      //speed of arm movement

    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");
        motorCenter = hwMap.get(DcMotor.class, "motorCenter");
        verticalarm = hwMap.get(DcMotor.class, "verticalarm");
        horizontalarm = hwMap.get(DcMotor.class,"horizantalarm");

        // Define Servos
        vaServoRight = hwMap.get(Servo.class, "vaServoRight");
        vaServoLeft = hwMap.get(Servo.class, "vaServoLeft");
        haServo = hwMap.get(Servo.class, "haServo");
        colorServo = hwMap.get(Servo.class, "colorServo");
        haWrist = hwMap.get(Servo.class, "haWrist");

        floorColor = hwMap.get(ColorSensor.class, "floorColor");
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensorColor");

        touchArm = hwMap.get(Servo.class, "touchArm");
        touchRed = hwMap.get(DigitalChannel.class, "touchRed");
        touchBlue = hwMap.get(DigitalChannel.class, "touchBlue");

        //Setting Motor Directions
        motorRight.setDirection(DcMotor.Direction.REVERSE); //right and left should be correct
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorCenter.setDirection(DcMotor.Direction.REVERSE);
        verticalarm.setDirection(DcMotor.Direction.FORWARD);
        horizontalarm.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorCenter.setPower(0);
        verticalarm.setPower(0);
        horizontalarm.setPower(0);

        //Initialize servo pos
        /*
        vaServoRight.setPosition(.6); //open
        vaServoLeft.setPosition(.7);  //open
        */
        haServo.setPosition(0);
        haWrist.setPosition(1); //init down
        colorServo.setPosition(1); //init in
        touchArm.setPosition(1); //init in

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
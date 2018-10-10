//THIS IS A MOTORHEADS PROGRAM

package legacy;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class FoOHardwareREV
{
    //movement wheels
    public DcMotor left; //front left
    public DcMotor right; //front right
    public DcMotor center; //center

    //not yet added

    //arms
    public DcMotor verticalarm;
    public DcMotor horizontalArm;
    public Servo haServo;
    public Servo vaServoRightT;
    public Servo vaServoLeftT;
    //public Servo haWrist;
    public DcMotor horzAdj;
    public Servo vaServoRightB;
    public Servo vaServoLeftB;

    //public Servo winchTest;


    //various sensors for autonomous and their servos
    //reading jewel colors
    public ColorSensor sensorColor;
    public Servo colorServo;

    public ColorSensor sensorColorRS;
    public Servo colorServoRS;

    public DigitalChannel glyphsen;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define Motors
        left = hwMap.get(DcMotor.class, "leftMotor");
        right = hwMap.get(DcMotor.class, "rightMotor");
        center = hwMap.get(DcMotor.class, "centerMotor");
        verticalarm = hwMap.get(DcMotor.class, "verticalArm");
        horzAdj = hwMap.get(DcMotor.class, "horzAdj");
        horizontalArm = hwMap.get(DcMotor.class, "horizontalArm");

        // Define Servos
        vaServoRightT = hwMap.get(Servo.class, "vaServoRightT");
        vaServoLeftT = hwMap.get(Servo.class, "vaServoLeftT");
        vaServoLeftB = hwMap.get(Servo.class, "vaServoLeftB");
        vaServoRightB = hwMap.get(Servo.class, "vaServoRightB");
        haServo = hwMap.get(Servo.class, "haServo");
        colorServo = hwMap.get(Servo.class, "colorServo");
        colorServoRS = hwMap.get(Servo.class, "colorServoRS");
        //haWrist = hwMap.get(Servo.class, "haWrist");

        // Define color sensors
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorColorRS = hwMap.get(ColorSensor.class, "sensorColorRS");
        glyphsen = hwMap.get(DigitalChannel.class, "glyphsen");



        //winchTest = hwMap.get(Servo.class, "winchTest");

        //Setting Motor Directions
        right.setDirection(DcMotor.Direction.REVERSE);

        left.setDirection(DcMotor.Direction.FORWARD);

        center.setDirection(DcMotor.Direction.FORWARD);
        verticalarm.setDirection(DcMotor.Direction.FORWARD);
        horzAdj.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        right.setPower(0);
        left.setPower(0);
        center.setPower(0);
        verticalarm.setPower(0);
        horzAdj.setPower(0);
        horizontalArm.setPower(0);
        //winchTest.setPosition(0);

        colorServo.setDirection(Servo.Direction.FORWARD);

        haServo.setPosition(0);
        //haWrist.setPosition(1); //init down
        colorServo.setPosition(1); //init in
        colorServoRS.setPosition(1); //init in

        // set the digital channel to input.
        glyphsen.setMode(DigitalChannel.Mode.INPUT);




        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horzAdj.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
